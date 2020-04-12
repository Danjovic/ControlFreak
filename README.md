# ControlFreak
Collection of firmware and schematics for connecting retro Joysticks and Gamepads to USB port.

The base circuit uses a cheap Arduino Pro Micro that come up with an ATMega32U4 chip that yields native USB connectivity.

|![Pro Micro](/doc/ProMicro.png)|
|:--:|
| *Arduino Pro Micro* |

Such board combined with [HID-Project](https://github.com/NicoHood/HID) provides classes for several HID devices including a resourceful game controller with 6 analog axes, 32 buttons and a 2 Dpads.

![Button Mapping](/doc/buttons.png)|
|:--:|
| *Buttons and Axes on Windows* |

![Button Mapping - Linux](/doc/buttonsOnLinux.png)|
|:--:|
| *Buttons and Axes on Linux* |

