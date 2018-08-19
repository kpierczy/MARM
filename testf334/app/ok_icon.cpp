/*
 * ok_icon.cpp
 *
 *  Created on: 4 lis 2013
 *      Author: lucck
 */

#include "resources.hpp"


namespace app {
namespace res {


//
//  Image data for ok2
//

static constexpr unsigned char ok2Bitmaps[] =
{
	//                          #
	//                         ####
	//                        #######
	//                       #########
	//                      #########
	//                     #########
	//                    #########
	//                   #########
	//                   ########
	//                  #########
	//                 #########
	//                #########
	//                ########
	//               #########
	//    ##         ########
	//    ###       ########
	//   #####     #########
	//  #######    ########
	//  ########  #########
	// ########## ########
	//  ##################
	//  #################
	//   ################
	//     #############
	//      ############
	//       ###########
	//        #########
	//          #######
	//           ######
	//             ###
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF, 0xFE, 0x7E, 0x3C, 0x1C, 0x08,
	0x00, 0x00, 0x00, 0xC0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x0F, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x08, 0x3E, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x1F, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x07, 0x0F, 0x1F, 0x1F, 0x3F, 0x3F, 0x3F, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// Bitmap sizes for ok2
const periph::display::icon_t ok_icon
{
	4,
	31,
	ok2Bitmaps
};



}}


