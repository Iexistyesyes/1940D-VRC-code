#include "autonSelector.hpp"

// ============================================================================
//  MATCH SCREENS  -  pictures / animations the brain can show during a match
//  (picked on the DISPLAY tab)
//
//  To add one:
//    1. run   python tools/make_match_image.py my_picture.png --name my_picture
//       (works with .png, .jpg and animated .gif, it makes src/assets/my_picture.c)
//       Always use the tool for GIFs: it works round a bug in the brain's GIF player.
//    2. add   LV_IMAGE_DECLARE(my_picture);   below
//    3. add   {"MY PICTURE", &my_picture},    to the list
//       (for a GIF put true on the end:  {"MY GIF", &my_gif, true}, )
//
//  The first one in the list is the one that's picked when the program starts.
// ============================================================================

extern "C" {
LV_IMAGE_DECLARE(match_logo);
}

namespace selector {

const std::vector<MatchScreen> matchScreens = {
    {"OVERRIDE LOGO", &match_logo},
};

} // namespace selector
