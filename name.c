#include "usb_names.h"

#define MIDI_NAME   {'A','P','K',' ','M','I','D','I'}
#define MIDI_NAME_LEN  8

// Do not change this part.  This exact format is required by USB.

struct usb_string_descriptor_struct usb_string_product_name = {
        2 + MIDI_NAME_LEN * 2,
        3,
        MIDI_NAME
};
