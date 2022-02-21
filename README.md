# TinBus

After many years of messing around getting different electronic devices and projects to talk to each other I came up with this wish list of features…

1. Simple, like really simple! Just 2 wires for both data and power.
2. Just add extra devices to the two wire bus.
3. Uses low cost and easily available components.
4. Very low active power and no quiescent power consumption.
5. Doesn’t require an accurate frequency reference.
6. Reliable in noisy environments. 
7. Includes error detection and correction mechanisms.
8. Range of more than 100 meters.
9. Easily isolated with no ground loop or system voltage issues.
10. Low data bandwidth of about 5000 bits per second.
11. Open source and free to use.

Of course, why reinvent the wheel – there are already so many excellent and widely used ways of connecting things together.

TinBus uses short electrical pulses to send and receive information across a single pair of wires. The timing of the pulses carries the information. So why not call it Temporal Impulse Networking bus (TIN bus). Most significantly it can be implemented on a low cost micro controller using only two pins, it uses very little power. Ideal for low power sensor nodes on the edge of IoT systems where wireless technology is not appropriate or where providing a copper connection is appropriate.

By sending very short pulses TinBus signals can be superimposed onto a DC power supply allowing power and data to be carried on a single pair of wires. It’s great for low power remote sensors.

TinBus is not only a point to point communication system. It allows for many connected devices. It also allows for multiple masters on a single pair of wires. To do this it requires collision detection and / or avoidance mechanisms in the line protocol. It adopts some techniques from CAN bus to achieve this.

Each byte is composed of 17 pulse positions, with the 9 odd numbered positions corresponding to clock pulses and the 8 even numbered positions corresponding to data pulses. The time between pulse positions is 100 us, and therefore 200 us between each clock pulse. Clock positions are required to contain a pulse, data positions are not. If there is a pulse present in a data position, it is representative of a logic zero, whereas the lack of a pulse is representative of a logic one. For each byte eight data bits are sent with the most significant bit sent first. A break between bytes is represented by at least 400 us without a pulse. The maximum continuous data rate is 500 bytes per second (5000 bps). A break between frames is represented by at least 800 us without a pulse. Extending the break between frames can be used to implement priority access to the bus. The speed of the bus can be changed by scaling the pulse timings. The implementation provided here uses software based bit banging so there is a trade off between bus speed and processor usage.
