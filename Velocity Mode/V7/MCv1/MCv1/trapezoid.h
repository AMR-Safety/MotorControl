//trapizoid.h

#ifndef TRAPEZOID_H
#define TRAPEZOID_H

#include <avr/io.h>

/**.
 *
 * @param target omega    Target angular velocity in rad/s
 */
void ramp_up_trapezoid(float target_omega);
void ramp_down_trapezoid(float target_omega);

#endif
