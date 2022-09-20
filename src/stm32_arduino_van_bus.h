#pragma once

#ifndef _stm32_arduino_van_bus_h
#define _stm32_arduino_van_bus_h

#include <Arduino.h>

//#define CIRCULAR_BUFFER_INT_SAFE
#include <RingBuf.h>

#ifdef __cplusplus
extern "C" {
#endif
    /* Include C code here */
    typedef enum {
        VAN_LINE_LEVEL_LOW = 0,
        VAN_LINE_LEVEL_HIGH = 1,
    } VAN_LINE_LEVEL;

    typedef enum {
        // the values are used for the timings
        VAN_NETWORK_TYPE_BODY = 16,
        VAN_NETWORK_TYPE_COMFORT = 8,
    } VAN_NETWORK_TYPE;

    typedef enum VAN_STATE
    {
        VAN_SEARCH = 0,
        VAN_BOF1 = 1,
        VAN_BOF2 = 2,
        VAN_BOF3 = 3,
        VAN_BOF4 = 4,
        VAN_ID = 5,
        VAN_COM = 6,
        VAN_DATA = 7,
        //  VAN_EOD = 8,
        VAN_ACK = 9,
        VAN_DONE = 10,
    };

    typedef enum VAN_TX_STATE
    {
        VAN_WAITING,
        VAN_SENDING,
    };

    typedef struct
    {
        uint8_t DataLen;
        uint8_t Data[32];
        uint8_t Ack;
    } VAN_FRAME;
#ifdef __cplusplus
}
#endif

#if defined(ARDUINO_ARCH_STM32)
#define VAN_RAM_ATTR
#endif

#if defined(ARDUINO_ARCH_ESP32)
#define VAN_RAM_ATTR ICACHE_RAM_ATTR
#endif

//RX
#define Bit_SET 1
#define Bit_RESET 0

//TX
#define VAN_READ_BUS() (digitalRead(_rxPin))
#define BIT_ARRAY_GET(Array,Bit) (*((Array) + (Bit)))
#define BIT_ARRAY_SET(Array,Bit) (*((Array) + (Bit))= 1)
#define BIT_ARRAY_CLR(Array,Bit) (*((Array) + (Bit))= 0)

#define VAN_TX_CLR() (digitalWrite(_txPin, LOW))
#define VAN_TX_SET() (digitalWrite(_txPin, HIGH))
#define VAN_TX_MAX_BIT_COUNT (35 * 10) // SOF(1) + IDEN(3) + COM(1) + DATA(max 28) + CRC(2)
#define VAN_TX_RETRY_COUNT 1

class Stm32ArduinoVanBus
{
    const uint16_t VAN_CRC15_TABLE[256] =
    {
      0x0000, 0x0F9D, 0x1F3A, 0x10A7, 0x3E74, 0x31E9, 0x214E, 0x2ED3,
      0x7CE8, 0x7375, 0x63D2, 0x6C4F, 0x429C, 0x4D01, 0x5DA6, 0x523B,
      0xF64D, 0xF9D0, 0xE977, 0xE6EA, 0xC839, 0xC7A4, 0xD703, 0xD89E,
      0x8AA5, 0x8538, 0x959F, 0x9A02, 0xB4D1, 0xBB4C, 0xABEB, 0xA476,
      0xE307, 0xEC9A, 0xFC3D, 0xF3A0, 0xDD73, 0xD2EE, 0xC249, 0xCDD4,
      0x9FEF, 0x9072, 0x80D5, 0x8F48, 0xA19B, 0xAE06, 0xBEA1, 0xB13C,
      0x154A, 0x1AD7, 0x0A70, 0x05ED, 0x2B3E, 0x24A3, 0x3404, 0x3B99,
      0x69A2, 0x663F, 0x7698, 0x7905, 0x57D6, 0x584B, 0x48EC, 0x4771,
      0xC993, 0xC60E, 0xD6A9, 0xD934, 0xF7E7, 0xF87A, 0xE8DD, 0xE740,
      0xB57B, 0xBAE6, 0xAA41, 0xA5DC, 0x8B0F, 0x8492, 0x9435, 0x9BA8,
      0x3FDE, 0x3043, 0x20E4, 0x2F79, 0x01AA, 0x0E37, 0x1E90, 0x110D,
      0x4336, 0x4CAB, 0x5C0C, 0x5391, 0x7D42, 0x72DF, 0x6278, 0x6DE5,
      0x2A94, 0x2509, 0x35AE, 0x3A33, 0x14E0, 0x1B7D, 0x0BDA, 0x0447,
      0x567C, 0x59E1, 0x4946, 0x46DB, 0x6808, 0x6795, 0x7732, 0x78AF,
      0xDCD9, 0xD344, 0xC3E3, 0xCC7E, 0xE2AD, 0xED30, 0xFD97, 0xF20A,
      0xA031, 0xAFAC, 0xBF0B, 0xB096, 0x9E45, 0x91D8, 0x817F, 0x8EE2,
      0x9CBB, 0x9326, 0x8381, 0x8C1C, 0xA2CF, 0xAD52, 0xBDF5, 0xB268,
      0xE053, 0xEFCE, 0xFF69, 0xF0F4, 0xDE27, 0xD1BA, 0xC11D, 0xCE80,
      0x6AF6, 0x656B, 0x75CC, 0x7A51, 0x5482, 0x5B1F, 0x4BB8, 0x4425,
      0x161E, 0x1983, 0x0924, 0x06B9, 0x286A, 0x27F7, 0x3750, 0x38CD,
      0x7FBC, 0x7021, 0x6086, 0x6F1B, 0x41C8, 0x4E55, 0x5EF2, 0x516F,
      0x0354, 0x0CC9, 0x1C6E, 0x13F3, 0x3D20, 0x32BD, 0x221A, 0x2D87,
      0x89F1, 0x866C, 0x96CB, 0x9956, 0xB785, 0xB818, 0xA8BF, 0xA722,
      0xF519, 0xFA84, 0xEA23, 0xE5BE, 0xCB6D, 0xC4F0, 0xD457, 0xDBCA,
      0x5528, 0x5AB5, 0x4A12, 0x458F, 0x6B5C, 0x64C1, 0x7466, 0x7BFB,
      0x29C0, 0x265D, 0x36FA, 0x3967, 0x17B4, 0x1829, 0x088E, 0x0713,
      0xA365, 0xACF8, 0xBC5F, 0xB3C2, 0x9D11, 0x928C, 0x822B, 0x8DB6,
      0xDF8D, 0xD010, 0xC0B7, 0xCF2A, 0xE1F9, 0xEE64, 0xFEC3, 0xF15E,
      0xB62F, 0xB9B2, 0xA915, 0xA688, 0x885B, 0x87C6, 0x9761, 0x98FC,
      0xCAC7, 0xC55A, 0xD5FD, 0xDA60, 0xF4B3, 0xFB2E, 0xEB89, 0xE414,
      0x4062, 0x4FFF, 0x5F58, 0x50C5, 0x7E16, 0x718B, 0x612C, 0x6EB1,
      0x3C8A, 0x3317, 0x23B0, 0x2C2D, 0x02FE, 0x0D63, 0x1DC4, 0x1259,
    };

    //RX related
    VAN_FRAME VAN_RX_Frame;
    RingBuf<VAN_FRAME, 10> RxQueue;

    VAN_STATE VAN_State = VAN_SEARCH;
    VAN_LINE_LEVEL _vanLineLevel;
    VAN_NETWORK_TYPE _vanNetworkType;

    uint8_t _rxPin;
    uint8_t _ledPin;
    uint8_t VAN_RX_BitCount = 0;
    uint32_t VAN_RX_LineData = 0;
    uint32_t VAN_RX_PreTime = 0;
    uint8_t DataLen = 0;
    uint8_t VAN_RX_LastInputPinState = HIGH;

    //TX related
    bool isBusFree = false;
    VAN_TX_STATE VAN_TX_State = VAN_WAITING;
    uint8_t _txPin;
    uint32_t VAN_TX_RetryTime = 0;
    uint32_t VAN_Bus_Idle_Count = 0;

    uint32_t VAN_TX_WaitReply = 0;
    uint32_t VAN_TX_BitCount = 0;
    uint32_t VAN_TX_BitIndex = 0;

    uint8_t VAN_TX_LastBitState = HIGH;
    uint8_t VAN_TX_Buffer[VAN_TX_MAX_BIT_COUNT];

    // Methods
    uint16_t GetCrc15(const uint8_t message[], uint8_t messageLength);

    // RX related
    void InitRXTimer();
    void StopRXTimer();
    VAN_RAM_ATTR void HandleRXInterrupt(uint32_t CurTime);

    // TX related
    void InitTXTimer();
    void StopTXTimer();
    VAN_RAM_ATTR void HandleTXInterrupt();

    friend void VAN_RX_EXTI_ISR();
    friend void VAN_TX_Timer_IT_Callback();
public:
    /*
     * Initialize the object
     * rxPin: receive pin
     * txPin: transmit pin
     * ledPin: if not 0 then the receiver toggles the supplied pin on reception
     * vanLineLevel: if a transceiver not used we can set which VAN level the pin is connected to
     * vanNetworkType: set the network type to either COMFORT for the 125 kbit/sec bus or BODY for the 62.5 kbit/sec bus
     */
    void Init(uint8_t rxPin, uint8_t txPin, uint8_t ledPin, VAN_LINE_LEVEL vanLineLevel, VAN_NETWORK_TYPE vanNetworkType);

    /*
     * Receive a VAN frame in the array and its length
     * Returns : true if we had an item to return otherwise false
     */
    bool Receive(uint8_t message[], uint8_t* messageLength);

    /*
     * Checks if the CRC part of the message is correct
     * Returns : true if we when success otherwise false
     */
    bool IsCrcOk(const uint8_t message[], uint8_t messageLength);

    /*
     * Sends a "Normal Data Frame" on the bus with or without requesting acknowledge
     * Returns: true if transmit started (doesn't guarantee if it was successful)
     *          false if another transmit is already in progress
     */
    bool SendNormalFrame(uint16_t iden, const uint8_t message[], uint8_t messageLength, uint8_t requireAck);

    /*
     * Send a "Reply Request Frame" on the bus.
     * The message will be completed with the DATA and CRC part by the requested module
     * Returns: true if transmit started (doesn't guarantee if it was successful)
     *          false if another transmit is already in progress
     */
    bool SendQueryFrame(uint16_t iden);

    /*
     * Send a "Reply Request Frame with Deferred Reply" on the bus.
     * The answer will be sent later by the requested module
     * Returns: true if transmit started (doesn't guarantee if it was successful)
     *          false if another transmit is already in progress
     */
    bool SendQueryFrameForDeferredReply(uint16_t iden);

};
#endif
