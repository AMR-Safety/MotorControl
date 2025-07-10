#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <dirent.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <chrono>
#include <signal.h>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>

// Helper macros for bit testing (hardware mode)
#define BITS_PER_LONG (sizeof(long) * 8)
#define BITS_TO_LONGS(nr) (((nr) + BITS_PER_LONG - 1) / BITS_PER_LONG)
#define test_bit(bit, array) ((array)[(bit) / BITS_PER_LONG] & (1L << ((bit) % BITS_PER_LONG)))

// Input modes
enum InputMode {
    MODE_HARDWARE,
    MODE_REMOTE
};

// Global state variables
std::atomic<bool> running(true);
std::atomic<bool> key_up_pressed(false);
std::atomic<bool> key_down_pressed(false);
std::atomic<bool> key_left_pressed(false);
std::atomic<bool> key_right_pressed(false);
int serial_fd = -1;
int input_fd = -1;
InputMode current_mode = MODE_HARDWARE;
struct termios orig_termios;

// Cleanup function
void cleanup() {
    if (serial_fd >= 0) {
        // Send stop command before closing
        write(serial_fd, "S", 1);
        close(serial_fd);
    }
    if (input_fd >= 0) {
        close(input_fd);
    }
    if (current_mode == MODE_REMOTE) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
    }
    running = false;
}

// Signal handler for graceful shutdown
void signal_handler(int sig) {
    std::cout << "\nShutting down gracefully...\n";
    cleanup();
    exit(0);
}

// ===== SHARED RS485 COMMUNICATION FUNCTIONS =====

int open_serial(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Failed to open serial port: " << port << std::endl;
        std::cerr << "Error: " << strerror(errno) << std::endl;
        std::cerr << "Note: Serial port is optional for testing input devices" << std::endl;
        return -1;
    }
    
    struct termios tty{};
    tcgetattr(fd, &tty);
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;
    tcsetattr(fd, TCSANOW, &tty);
    
    return fd;
}

void send_command(char cmd) {
    if (serial_fd >= 0) {
        write(serial_fd, &cmd, 1);

        // Read response
        char response[32];
        int n = read(serial_fd, response, sizeof(response) - 1);
        if (n > 0) {
            response[n] = '\0';
            std::cout << "Command '" << cmd << "' � Response: " << response;
        }
    } else {
        std::cout << "Command '" << cmd << "' � (Serial not connected)" << std::endl;
    }
}

// ===== HARDWARE MODE FUNCTIONS =====

std::string get_device_name(const std::string& device_path) {
    int fd = open(device_path.c_str(), O_RDONLY);
    if (fd < 0) return "Unknown";
    
    char name[256] = "Unknown";
    if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0) {
        strcpy(name, "Unknown");
    }
    close(fd);
    return std::string(name);
}

bool has_arrow_keys(const std::string& device_path) {
    int fd = open(device_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) return false;
    
    unsigned long evbit = 0;
    if (ioctl(fd, EVIOCGBIT(0, sizeof(evbit)), &evbit) >= 0) {
        if (evbit & (1 << EV_KEY)) {
            unsigned long keybit[BITS_TO_LONGS(KEY_MAX)] = {0};
            if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybit)), keybit) >= 0) {
                bool has_arrows = test_bit(KEY_UP, keybit) && 
                                test_bit(KEY_DOWN, keybit) && 
                                test_bit(KEY_LEFT, keybit) && 
                                test_bit(KEY_RIGHT, keybit);
                close(fd);
                return has_arrows;
            }
        }
    }
    close(fd);
    return false;
}

void list_input_devices() {
    const char* input_dir = "/dev/input";
    DIR* dir = opendir(input_dir);
    if (!dir) {
        std::cerr << "Cannot open " << input_dir << std::endl;
        return;
    }
    
    std::cout << "\n=== Available Input Devices with Arrow Keys ===\n";
    
    struct dirent* entry;
    std::vector<std::string> event_devices;
    
    while ((entry = readdir(dir)) != nullptr) {
        if (strncmp(entry->d_name, "event", 5) == 0) {
            std::string device_path = std::string(input_dir) + "/" + entry->d_name;
            event_devices.push_back(device_path);
        }
    }
    closedir(dir);
    
    std::sort(event_devices.begin(), event_devices.end());
    
    for (const auto& device : event_devices) {
        if (has_arrow_keys(device)) {
            std::string name = get_device_name(device);
            std::cout << device << " - " << name << std::endl;
        }
    }
    std::cout << "\n";
}

std::string find_keyboard_device() {
    const char* input_dir = "/dev/input";
    DIR* dir = opendir(input_dir);
    if (!dir) {
        std::cerr << "Cannot open " << input_dir << std::endl;
        return "";
    }
    
    struct dirent* entry;
    std::vector<std::string> event_devices;
    
    while ((entry = readdir(dir)) != nullptr) {
        if (strncmp(entry->d_name, "event", 5) == 0) {
            std::string device_path = std::string(input_dir) + "/" + entry->d_name;
            event_devices.push_back(device_path);
        }
    }
    closedir(dir);
    
    std::sort(event_devices.begin(), event_devices.end());
    std::reverse(event_devices.begin(), event_devices.end());
    
    for (const auto& device : event_devices) {
        if (has_arrow_keys(device)) {
            std::cout << "Found keyboard device: " << device;
            std::cout << " - " << get_device_name(device) << std::endl;
            return device;
        }
    }
    
    std::cerr << "No suitable keyboard device found!" << std::endl;
    return "";
}

int open_input_device(const std::string& specified_device = "") {
    std::string device;
    
    if (!specified_device.empty()) {
        device = specified_device;
        if (device.find("/dev/input/") != 0) {
            device = "/dev/input/" + device;
        }
        std::cout << "Using specified device: " << device;
        std::cout << " - " << get_device_name(device) << std::endl;
    } else {
        device = find_keyboard_device();
        if (device.empty()) {
            std::cerr << "Error: Could not find keyboard device." << std::endl;
            std::cerr << "Try specifying a device manually or run with sudo." << std::endl;
            return -1;
        }
    }
    
    int fd = open(device.c_str(), O_RDONLY);
    if (fd < 0) {
        perror("Failed to open input device");
        std::cerr << "Device: " << device << std::endl;
        std::cerr << "Try running with sudo: sudo ./your_program" << std::endl;
        return -1;
    }
    
    return fd;
}

void process_hardware_input_events() {
    struct input_event ev;
    
    while (running) {
        ssize_t bytes = read(input_fd, &ev, sizeof(ev));
        
        if (bytes == sizeof(ev)) {
            if (ev.type == EV_KEY) {
                bool key_down = (ev.value == 1);
                bool key_repeat = (ev.value == 2);
                
                if (key_repeat) continue;
                
                std::string action = key_down ? "PRESSED" : "RELEASED";
                
                switch (ev.code) {
                    case KEY_UP:
                        key_up_pressed = key_down;
                        std::cout << "� Arrow " << action << std::endl;
                        break;
                        
                    case KEY_DOWN:
                        key_down_pressed = key_down;
                        std::cout << "� Arrow " << action << std::endl;
                        break;
                        
                    case KEY_LEFT:
                        key_left_pressed = key_down;
                        std::cout << "� Arrow " << action << std::endl;
                        break;
                        
                    case KEY_RIGHT:
                        key_right_pressed = key_down;
                        std::cout << "� Arrow " << action << std::endl;
                        break;
                        
                    case KEY_Q:
                        if (key_down) {
                            std::cout << "Q pressed - Quitting..." << std::endl;
                            running = false;
                        }
                        break;
                    
                    case KEY_W:
                        key_up_pressed = key_down;
                        key_down_pressed = false;
                        std::cout << "W (hardware) " << action << std::endl;
                        break;

                    case KEY_A:
                        key_left_pressed = key_down;
                        key_right_pressed = false;
                        std::cout << "A (hardware) " << action << std::endl;
                        break;

                    case KEY_S:
                        // Only do backward if not used for stop in your key mapping!
                        key_down_pressed = key_down;
                        key_up_pressed = false;
                        std::cout << "S (hardware) BACKWARD " << action << std::endl;
                        break;

                    case KEY_D:
                        key_right_pressed = key_down;
                        key_left_pressed = false;
                        std::cout << "D (hardware) " << action << std::endl;
                        break;

                    case KEY_F:
                    case KEY_SPACE:
                        if (key_down) {
                            std::cout << "EMERGENCY STOP (hardware F key)!" << std::endl;
                            key_up_pressed = false;
                            key_down_pressed = false;
                            key_left_pressed = false;
                            key_right_pressed = false;
                            send_command('S');
                        }
                        break;
                }
            }
        }
        else if (bytes < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "Input device read error: " << strerror(errno) << std::endl;
                running = false;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// ===== REMOTE MODE FUNCTIONS =====

void reset_terminal_mode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

void set_raw_mode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(reset_terminal_mode);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

void process_arrow_key(char direction_char) {
    std::string direction_name = "";
    
    switch (direction_char) {
        case 'A':  // Up arrow
            key_up_pressed = true;
            key_down_pressed = false;
            direction_name = "FORWARD";
            break;
        case 'B':  // Down arrow
            key_down_pressed = true;
            key_up_pressed = false;
            direction_name = "BACKWARD";
            break;
        case 'C':  // Right arrow
            key_right_pressed = true;
            key_left_pressed = false;
            direction_name = "RIGHT";
            break;
        case 'D':  // Left arrow
            key_left_pressed = true;
            key_right_pressed = false;
            direction_name = "LEFT";
            break;
    }
    
    if (!direction_name.empty()) {
        std::cout << "� " << direction_name << " (hold to continue, release to stop)\n";
    }
}

// === NEW: Macro and Velocity helpers ===

// Simple macro: Forward � Right � Forward � Stop
void run_macro_sequence() {
    std::cout << "[Macro] Forward � Right � Forward � Stop\n";
    send_command('F');
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    send_command('R');
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    send_command('F');
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    send_command('S');
}

// Prompt for V: velocity, angular, time
void prompt_and_send_velocity_command() {
    float v = 0, w = 0, t = 0;
    std::string line;
    std::cout << "Enter velocity command (linear angular duration): ";
    std::getline(std::cin, line);
    if (sscanf(line.c_str(), "%f %f %f", &v, &w, &t) == 3) {
        char buf[64];
        snprintf(buf, sizeof(buf), "V:%.2f,%.2f,%.2f\n", v, w, t);
        std::cout << "Sending: " << buf;
        if (serial_fd >= 0) write(serial_fd, buf, strlen(buf));
    } else {
        std::cout << "Invalid format. Example: 100 0 2\n";
    }
}

void process_remote_input_events() {
    while (running) {
        char c;
        int bytes_read = read(STDIN_FILENO, &c, 1);

        if (bytes_read > 0) {
            if (c == '\x1b') {  // ESC character (arrow keys)
                char seq[2];
                if (read(STDIN_FILENO, &seq[0], 1) > 0 && seq[0] == '[') {
                    if (read(STDIN_FILENO, &seq[1], 1) > 0) {
                        process_arrow_key(seq[1]);
                    }
                }
            }
            // WASD and F (Option 2 logic)
            else if (c == 'w' || c == 'W') {
                key_up_pressed = true;
                key_down_pressed = false;
                std::cout << "W � FORWARD\n";
            }
            else if (c == 'a' || c == 'A') {
                key_left_pressed = true;
                key_right_pressed = false;
                std::cout << "A � LEFT\n";
            }
            else if (c == 's' || c == 'S') {
                key_down_pressed = true;
                key_up_pressed = false;
                std::cout << "S � BACKWARD\n";
            }
            else if (c == 'd' || c == 'D') {
                key_right_pressed = true;
                key_left_pressed = false;
                std::cout << "D � RIGHT\n";
            }
            else if (c == 'f' || c == 'F') {
                std::cout << "EMERGENCY STOP (F key)!\n";
                key_up_pressed = false;
                key_down_pressed = false;
                key_left_pressed = false;
                key_right_pressed = false;
                send_command('S');
            }
            // ----------- Velocity Command Prompt -----------
            else if (c == 'v' || c == 'V') {
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                prompt_and_send_velocity_command();
            }
            // ----------- Macro -----------
            else if (c == 'm' || c == 'M') {
                run_macro_sequence();
            }
            // ----------- Quit -----------
            else if (c == 'q' || c == 'Q') {
                std::cout << "Quitting...\n";
                running = false;
                break;
            }
            // ----------- Space: Stop -----------
            else if (c == ' ') {
                std::cout << "Stop\n";
                key_up_pressed = false;
                key_down_pressed = false;
                key_left_pressed = false;
                key_right_pressed = false;
            }
        } else {
            // No key pressed, stop movement if necessary
            if (key_up_pressed || key_down_pressed || key_left_pressed || key_right_pressed) {
                key_up_pressed = false;
                key_down_pressed = false;
                key_left_pressed = false;
                key_right_pressed = false;
                std::cout << "� Key released - stopping smoothly\n";
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ===== SHARED COMMAND SENDER THREAD =====

void command_sender_thread() {
    char current_command = 0;
    auto last_command_time = std::chrono::steady_clock::now();
    
    while (running) {
        auto now = std::chrono::steady_clock::now();
        auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time);
        
        // Determine current movement command based on key states
        char new_command = 0;
        
        // Check for diagonal combinations first (priority over single directions)
        if (key_up_pressed && key_right_pressed && !key_down_pressed && !key_left_pressed) {
            new_command = '1';  // Forward-Right diagonal
        }
        else if (key_up_pressed && key_left_pressed && !key_down_pressed && !key_right_pressed) {
            new_command = '2';  // Forward-Left diagonal
        }
        else if (key_down_pressed && key_right_pressed && !key_up_pressed && !key_left_pressed) {
            new_command = '3';  // Backward-Right diagonal
        }
        else if (key_down_pressed && key_left_pressed && !key_up_pressed && !key_right_pressed) {
            new_command = '4';  // Backward-Left diagonal
        }
        // Single direction movements
        else if (key_up_pressed && !key_down_pressed && !key_left_pressed && !key_right_pressed) {
            new_command = 'F';  // Forward only
        }
        else if (key_down_pressed && !key_up_pressed && !key_left_pressed && !key_right_pressed) {
            new_command = 'B';  // Backward only
        }
        else if (key_left_pressed && !key_right_pressed && !key_up_pressed && !key_down_pressed) {
            new_command = 'L';  // Turn left only
        }
        else if (key_right_pressed && !key_left_pressed && !key_up_pressed && !key_down_pressed) {
            new_command = 'R';  // Turn right only
        }
        else {
            new_command = 'S';  // Stop (no keys, conflicting keys, or invalid combinations)
        }
        
        // Send command if it changed
        if (new_command != current_command) {
            send_command(new_command);
            current_command = new_command;
            last_command_time = now;
            
            // Print current action
            switch (new_command) {
                case 'F': std::cout << "� MOVING FORWARD" << std::endl; break;
                case 'B': std::cout << "� MOVING BACKWARD" << std::endl; break;
                case 'L': std::cout << "� TURNING LEFT" << std::endl; break;
                case 'R': std::cout << "� TURNING RIGHT" << std::endl; break;
                case '1': std::cout << "� MOVING FORWARD-RIGHT" << std::endl; break;
                case '2': std::cout << "� MOVING FORWARD-LEFT" << std::endl; break;
                case '3': std::cout << "� MOVING BACKWARD-RIGHT" << std::endl; break;
                case '4': std::cout << "� MOVING BACKWARD-LEFT" << std::endl; break;
                case 'S': std::cout << "� STOPPED" << std::endl; break;
            }
        }
        // Send heartbeat if moving and enough time has passed
        else if (new_command != 'S' && time_since_last.count() >= 200) {
            send_command('H');
            last_command_time = now;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// ===== MAIN PROGRAM =====

void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]\n";
    std::cout << "\nOptions:\n";
    std::cout << "  -m hardware     Use hardware-level keyboard input (default)\n";
    std::cout << "  -m remote       Use remote/terminal keyboard input\n";
    std::cout << "  -d <device>     Specify input device (hardware mode only)\n";
    std::cout << "  -l              List available input devices (hardware mode)\n";
    std::cout << "  -s <port>       Specify serial port (default: /dev/ttyUSB0)\n";
    std::cout << "  -h              Show this help\n";
    std::cout << "\nExamples:\n";
    std::cout << "  " << program_name << " -l                    # List available devices\n";
    std::cout << "  " << program_name << " -m hardware -d event12 # Use hardware mode with event12\n";
    std::cout << "  " << program_name << " -m remote             # Use remote/SSH mode\n";
    std::cout << "  " << program_name << " -s /dev/ttyUSB1       # Use different serial port\n";
}

int main(int argc, char* argv[]) {
    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::string specified_device = "";
    std::string serial_port = "/dev/ttyUSB0";
    std::string mode_str = "hardware";
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            mode_str = argv[++i];
        }
        else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            specified_device = argv[++i];
        }
        else if (strcmp(argv[i], "-l") == 0) {
            list_input_devices();
            return 0;
        }
        else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            serial_port = argv[++i];
        }
        else if (strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        else {
            std::cerr << "Unknown option: " << argv[i] << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }
    
    // Set input mode
    if (mode_str == "hardware") {
        current_mode = MODE_HARDWARE;
    } else if (mode_str == "remote") {
        current_mode = MODE_REMOTE;
    } else {
        std::cerr << "Invalid mode: " << mode_str << ". Use 'hardware' or 'remote'." << std::endl;
        return 1;
    }
    
    std::cout << "=== Unified AMR Control System ===\n";
    std::cout << "Mode: " << (current_mode == MODE_HARDWARE ? "Hardware" : "Remote") << std::endl;
    
    // Initialize input method
    if (current_mode == MODE_HARDWARE) {
        std::cout << "Starting hardware-level input detection...\n";
        input_fd = open_input_device(specified_device);
        if (input_fd < 0) {
            std::cout << "\nUse -l to list available devices, or -d to specify device\n";
            return 1;
        }
    } else {
        std::cout << "Starting remote/terminal input detection...\n";
        set_raw_mode();
    }
    
    // Open serial port (optional)
    serial_fd = open_serial(serial_port.c_str());
    if (serial_fd < 0) {
        std::cout << "Continuing without serial port (input device testing mode)\n";
    }
    
    std::cout << "\n=== READY FOR CONTROL ===\n";
    std::cout << "Controls:\n";
    std::cout << "  � = Forward    � = Backward\n";
    std::cout << "  � = Turn Left  � = Turn Right\n";
    std::cout << "  �+� = Forward-Right   �+� = Forward-Left\n";
    std::cout << "  �+� = Backward-Right  �+� = Backward-Left\n";
    std::cout << "  Q = Quit       Space = Emergency Stop\n";
    std::cout << "  W/A/S/D = Forward/Left/Backward/Right\n";
    std::cout << "  F = Emergency Stop\n";
    std::cout << "  V = Velocity command\n";
    std::cout << "  M = Macro sequence\n";
    if (current_mode == MODE_HARDWARE) {
        std::cout << "Hardware input detection active\n";
        std::cout << "Safety: Invalid combinations = Automatic Stop\n";
        std::cout << "Note: Hold multiple keys simultaneously for diagonal movement\n\n";
    } else {
        std::cout << "Remote terminal input active\n";
        std::cout << "Note: Diagonal movement requires simultaneous key press\n";
        std::cout << "Hold arrow keys/WASD for continuous movement\n\n";
    }
    
    // Start input processing thread
    std::thread input_thread;
    if (current_mode == MODE_HARDWARE) {
        input_thread = std::thread(process_hardware_input_events);
    } else {
        input_thread = std::thread(process_remote_input_events);
    }
    
    // Start command sender thread
    std::thread sender_thread(command_sender_thread);
    
    // Main thread waits for shutdown
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Cleanup
    if (input_thread.joinable()) {
        input_thread.join();
    }
    if (sender_thread.joinable()) {
        sender_thread.join();
    }
    
    cleanup();
    return 0;
}