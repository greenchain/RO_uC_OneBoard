#ifndef BLYNK_RO_H_
#define BLYNK_RO_H_

#define BLYNK_PRINT Serial
#define APP_DEBUG
#define BLYNK_FIRMWARE_VERSION "1.0.1"
#define BLYNK_TEMPLATE_ID "TMPL22ySLWsWc"
#define BLYNK_TEMPLATE_NAME "RO"

#define B_STATE V0
#define B_PRODUCT_TANK V1
#define B_FEED_TANK V2
#define B_PREFILTER_LOCKOUT V3
// #define B_PH V2
// #define B_ORP V3
#define B_EC V4
#define B_TEMP V5
#define B_RECOVERY V6
#define B_RO_PUMP V7
#define B_BOOST_PUMP V8
#define B_EXTERNAL_FAULT V9

#define B_FEED_FLOW V10
#define B_PERM_FLOW V11
#define B_RECYCLE_FLOW V12
#define B_BRINE_FLOW V13

#define B_FEED_PRESS V20
#define B_HP_PRESS V21
#define B_POST_MEM_PRESS V22
#define B_DELTA_PRESS V23
#define B_BOOST_PRESS V24

#define B_PRODUCT_VOLUME V30

#define BL_STATE_CHANGE "state"
#define BL_WARNING "warn"
#define BL_FAULT "fault"
#define BL_SETTINGS "set"

#endif