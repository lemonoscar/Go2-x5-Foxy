
#ifndef GO2_X5_KEYBOARD_HPP
#define GO2_X5_KEYBOARD_HPP

#include <string>
#include <vector>
#include <unitree/idl/go2/WirelessController_.hpp>

namespace Go2X5Keyboard {

// Union for joystick keys
typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} KeySwitchUnion;

} // namespace Go2X5Keyboard

#endif // GO2_X5_KEYBOARD_HPP
