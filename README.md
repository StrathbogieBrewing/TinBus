# TinBus

After many years of messing around getting different electronic devices and projects to talk to each other I came up with this wish list of features…

1. Simple, like really simple! Easy to wire up, configure and use.
1. Requires low cost and easily available parts.
1. Low power consumption.
1. Doesn’t require an accurate frequency reference.
1. Reliable and tolerant of noisy environments.
1. Range of more than 100 meters.
1. Isolated, no ground loop or system voltage issues.
1. Able to connect many devices to the same system.
1. Low data bandwidth of about 2500 bits per second.
1. Open source and free to use.

Of course, why reinvent the wheel – there are already so many excellent and widely used ways of connecting things together.

TinBus uses short electrical pulses to send information across a pair of wires. The timing of the pulses carries the information. So why not call it Temporal Impulse Networking bus (TIN bus). It’s like a low speed mash up of Multi-point Low Voltage Digital Signalling (TIA/EIA-899), Ethernet Fast Link Pulses (IEEE 802.3) and CAN bus (ISO 11898). Most significantly it can be implemented on a low cost micro controller using only two pins, it uses very little power and its signal can be AC coupled.

By sending very short AC pulses TinBus signals can be isolated using capacitors or low cost Ethernet transformers. They can also be superimposed onto a power supply allowing power and data to be carried on a single pair of wires. It’s great for low power remote sensors. By using pulses that are short compared to the data rate the wires can be reversed without losing information.

TinBus is not only a point to point communication system. It allows for many connected devices. It also allows for multiple masters on a single pair of wires. To do this it requires collision detection and / or avoidance mechanisms in the line protocol. It adopts some techniques from CAN bus to achieve this and is able to transport CAN bus frames natively.

Each pulse is a symmetric waveform with no DC component. Pulses have a fundamental period of 1.5 us (667 kHz) and are composed of seven periods of 250 ns. The sequence is 0, +, +, 0, -, -, 0. This sequence helps minimize the total harmonic distortion, bandwidth and radiated emissions.

Each byte is composed of 17 pulse positions, with the 9 odd numbered positions corresponding to clock pulses and the 8 even numbered positions corresponding to data pulses. The time between pulse positions is 200 us, and therefore 400 us between each clock pulse. Clock positions are required to contain a pulse, data positions are not. If there is a pulse present in a data position, it is representative of a logic zero, whereas the lack of a pulse is representative of a logic one. For each byte eight data bits are sent with the most significant bit sent first. A break between bytes is represented by at least 800 us without a pulse. The maximum continuous data rate is 250 bytes per second (2500 bps). A break between frames is represented by at least 1600 us without a pulse. Extending the break between frames can be used to implement priority access to the bus. The speed of the bus can be changed by scaling the pulse timings. Given the implementation is in software using bit banging there is a trade off between bus speed and processor usage.
