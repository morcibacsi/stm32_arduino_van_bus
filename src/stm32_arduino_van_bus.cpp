#include "stm32_arduino_van_bus.h"

HardwareTimer* RX_Timer;
HardwareTimer* TX_Timer;

static Stm32ArduinoVanBus* thisVanDriver;

static uint8_t _round_to_nearest(uint8_t numToRound, uint8_t multiple)
{
    if (multiple == 0)
        return numToRound;

    uint8_t remainder = numToRound % multiple;
    if (remainder == 0)
        return numToRound;

    uint8_t next = numToRound + multiple - remainder;
    uint8_t prev = next - multiple;

    if (next - numToRound < numToRound - prev)
    {
        return next;
    }
    else
    {
        return prev;
    }
}

static void _BitArray_PutEManchester(uint8_t* Array, uint16_t BitIndex, uint8_t Data)
{
    uint8_t counter;
    uint8_t BitMask = 0x80;
    for (counter = 0; counter < 4; ++counter)
    {
        if (Data & BitMask)
        {
            BIT_ARRAY_SET(Array, BitIndex);
        }
        else
        {
            BIT_ARRAY_CLR(Array, BitIndex);
        }
        ++BitIndex;
        BitMask >>= 1;
    }

    if (Data & 0x10)
    {
        BIT_ARRAY_CLR(Array, BitIndex);
    }
    else
    {
        BIT_ARRAY_SET(Array, BitIndex);
    }
    ++BitIndex;

    for (counter = 4; counter < 8; ++counter)
    {
        if (Data & BitMask)
        {
            BIT_ARRAY_SET(Array, BitIndex);
        }
        else
        {
            BIT_ARRAY_CLR(Array, BitIndex);
        }
        ++BitIndex;
        BitMask >>= 1;
    }

    if (Data & 0x01)
    {
        BIT_ARRAY_CLR(Array, BitIndex);
    }
    else
    {
        BIT_ARRAY_SET(Array, BitIndex);
    }
}

void VAN_RX_Timer_IT_Callback(void)
{
}

void VAN_TX_Timer_IT_Callback(void)
{
    thisVanDriver->HandleTXInterrupt();
}

void VAN_RX_EXTI_ISR()
{
    uint32_t CurTime = RX_Timer->getCount();
    thisVanDriver->HandleRXInterrupt(CurTime);
}

void Stm32ArduinoVanBus::HandleRXInterrupt(uint32_t CurTime)
{
    if (VAN_TX_State == VAN_SENDING) {
        return;
    }

    uint32_t Delta;
    uint32_t Bits;
    uint32_t Bit;

    Bit = VAN_READ_BUS();
    
    VAN_RX_LastInputPinState = Bit;

    if (_vanLineLevel == VAN_LINE_LEVEL_LOW)
    {
        if (Bit == 0)
        {
            Bit = 1;
        }
        else
        {
            Bit = 0;
        }
    }

    //Delta = (CurTime - VAN_RX_PreTime);
    
    // we get better results when we first correct the elapsed time to multiple of 1 TS
    Delta = _round_to_nearest(CurTime - VAN_RX_PreTime, _vanNetworkType);
    
    VAN_RX_PreTime = CurTime;

    if (_vanNetworkType == VAN_NETWORK_TYPE_COMFORT)
    {
        Bits = (++Delta) >> 3;  // divide by 8
    }
    else if (_vanNetworkType == VAN_NETWORK_TYPE_BODY)
    {
        Bits = (++Delta) >> 4;  // divide by 16
    }

    isBusFree = false;

    if (0 == Bits)
    {
        return;
    }
    if (Bits >= 8)
    {
        isBusFree = true;//Bit && Bits >= 13;
        if (VAN_SEARCH != VAN_State)
        {
            VAN_State = VAN_SEARCH;
        }
    }

    switch (VAN_State)
    {
    case VAN_SEARCH:
    {
        VAN_RX_BitCount = 0;
        VAN_RX_LineData = 0;
        DataLen = 0;

        if (_ledPin) {
            digitalWrite(_ledPin, LOW);
        }

        // After more than 5 bits of high level
        if (Bits >= 6 && Bit_SET == Bit)
        {
            VAN_State = VAN_BOF1;
        }
    }
    break;
    case VAN_BOF1:
    {
        // After 4 bits of low level
        if (4 == Bits && Bit_RESET == Bit)
        {
            VAN_State = VAN_BOF2;
        }
        else
        {
            VAN_State = VAN_SEARCH;
        }
    }
    break;
    case VAN_BOF2:
    {
        // After 4 bits of high level
        if (4 == Bits && Bit == Bit_SET)
        {
            VAN_State = VAN_BOF3;
        }
        else
        {
            VAN_State = VAN_SEARCH;
        }
    }
    break;
    case VAN_BOF3:
    {
        // After 1 bit of low level
        if (1 == Bits && Bit_RESET == Bit)
        {
            VAN_State = VAN_BOF4;
        }
        else
        {
            VAN_State = VAN_SEARCH;
        }
    }
    break;
    case VAN_BOF4:
    {
        // After 1-5 bits of high level
        if (Bits <= 5 && Bit_SET == Bit)
        {
            VAN_State = VAN_ID;
            VAN_RX_BitCount = Bits - 1;
            VAN_RX_LineData = (1 << VAN_RX_BitCount) - 1;
            if (RxQueue.isFull())
            {
                VAN_State = VAN_SEARCH;
            }
        }
        else
        {
            VAN_State = VAN_SEARCH;
        }
    }
    break;

    case VAN_ID:
    {
        VAN_RX_LineData <<= Bits;
        if (Bit)
        {
            VAN_RX_LineData |= (1 << Bits) - 1;
        }
        else
        {
            VAN_RX_LineData &= ~((1 << Bits) - 1);
        }

        VAN_RX_BitCount += Bits;
        if (VAN_RX_BitCount >= 15)
        {
            uint32_t tmp;
            VAN_RX_BitCount -= 15;
            if (VAN_RX_BitCount)
            {
                tmp = VAN_RX_LineData >> VAN_RX_BitCount;
                VAN_RX_LineData &= ((1 << VAN_RX_BitCount) - 1);
            }
            else
            {
                tmp = VAN_RX_LineData;
                VAN_RX_LineData = 0;
            }

            VAN_RX_Frame.Data[0] = 0x0E;
            VAN_RX_Frame.Data[1] = ((tmp >> 11) << 4) | ((tmp & 0x3E0) >> 6);
            VAN_RX_Frame.Data[2] = ((tmp & 0x1F) >> 1) << 4;
            DataLen = 2;
            VAN_State = VAN_COM;
        }
    }
    break;
    case VAN_COM:
    {
        VAN_RX_LineData <<= Bits;
        if (Bit)
        {
            VAN_RX_LineData |= (1 << Bits) - 1;
        }
        else
        {
            VAN_RX_LineData &= ~((1 << Bits) - 1);
        }

        VAN_RX_BitCount += Bits;
        if (VAN_RX_BitCount >= 5)
        {
            uint32_t COM;
            VAN_RX_BitCount -= 5;
            if (VAN_RX_BitCount)
            {
                COM = VAN_RX_LineData >> (VAN_RX_BitCount + 1);
                VAN_RX_LineData &= ((1 << VAN_RX_BitCount) - 1);
            }
            else
            {
                COM = VAN_RX_LineData >> 1;
                VAN_RX_LineData = 0;
            }
            VAN_RX_Frame.Data[2] |= COM;

            //if(VAN_TX_WaitReply)
            {
                //VAN_TX_WaitReply = 0;
            }
            VAN_State = VAN_DATA;
        }
    }
    break;
    case VAN_DATA:
    {
        VAN_RX_LineData <<= Bits;
        if (Bit)
        {
            VAN_RX_LineData |= (1 << Bits) - 1;
        }
        else
        {
            VAN_RX_LineData &= ~((1 << Bits) - 1);
        }

        VAN_RX_BitCount += Bits;
        if (VAN_RX_BitCount >= 10)
        {
            uint32_t tmp;
            uint32_t Data;
            int32_t shift = VAN_RX_BitCount - 10;
            if ((VAN_RX_LineData >> shift) & 0x3)
            {
                VAN_RX_BitCount -= 10;
                if (VAN_RX_BitCount)
                {
                    tmp = VAN_RX_LineData >> VAN_RX_BitCount;
                    VAN_RX_LineData &= ((1 << VAN_RX_BitCount) - 1);
                }
                else
                {
                    tmp = VAN_RX_LineData;
                    VAN_RX_LineData = 0;
                }

                Data = (tmp >> 6) << 4;
                Data |= (tmp >> 1) & 0xF;

                VAN_RX_Frame.Data[DataLen + 1] = Data;
                ++DataLen;
            }
            else
            {
                VAN_RX_BitCount -= 10;

                if (VAN_RX_BitCount)
                {
                    tmp = VAN_RX_LineData >> VAN_RX_BitCount;
                    VAN_RX_LineData &= ((1 << VAN_RX_BitCount) - 1);
                }
                else
                {
                    tmp = VAN_RX_LineData;
                    VAN_RX_LineData = 0;
                }

                Data = (tmp >> 6) << 4;
                Data |= ((tmp >> 2) & 0x7) << 1;

                VAN_RX_Frame.Data[DataLen + 1] = Data;
                ++DataLen;

                //We reached the end of the message we can store its length
                VAN_RX_Frame.DataLen = DataLen + 1;

                VAN_State = VAN_ACK;
                if (DataLen >= 2 && DataLen < 32)
                {
                    if (_ledPin) {
                        digitalWrite(_ledPin, HIGH);
                    }

                    RxQueue.push(VAN_RX_Frame);
                    VAN_State = VAN_SEARCH;

                    // we calculate the CRC after reception, outside of the ISR
                    /*
                    const unsigned short CalculatedCRC = GetCrc15(VAN_RX_Frame.Data,VAN_RX_Frame.DataLen - 2);
                    unsigned short FrameCRC = (VAN_RX_Frame.Data[VAN_RX_Frame.DataLen-2] << 8) | (VAN_RX_Frame.Data[VAN_RX_Frame.DataLen-1]);

                    if(CalculatedCRC == FrameCRC)
                    {
                        RxQueue.push(VAN_RX_Frame);
                    }
                    else
                    {
                        VAN_State = VAN_SEARCH;
                    }
                    //*/
                }
                else
                {
                    VAN_State = VAN_SEARCH;
                }
            }
        }
    }
    break;
    case VAN_ACK:
    {
        if (!Bit)
        {
            VAN_State = VAN_SEARCH;
        }
    }
    break;
    default:
        break;
    }
}

void Stm32ArduinoVanBus::HandleTXInterrupt()
{
    if (VAN_TX_RetryTime) {
        if (VAN_TX_BitIndex < VAN_TX_BitCount) {
            if (VAN_TX_State == VAN_WAITING) {
              if (!isBusFree) // Wait until the bus is free
              {
                  return;
              }
                // Seems like the RX interrupt consumes so much processor power that we need to stop it, 
                // otherwise the timings for transmitting will be incorrect
                StopRXTimer();
            }
            VAN_TX_State = VAN_SENDING;
            if (VAN_TX_BitIndex > 0 && VAN_TX_BitIndex != VAN_TX_BitCount - 1)
            {
              ///*
                // After the first transmission we check every bit if we had a collision (excluding the last bit)
                VAN_RX_LastInputPinState = VAN_READ_BUS();
                if (VAN_RX_LastInputPinState != VAN_TX_LastBitState)
                {
                    VAN_TX_State = VAN_WAITING;
                    //VAN_TX_SET();
                    InitRXTimer();
                    VAN_TX_BitIndex = 0;
                    VAN_Bus_Idle_Count = 0;
                    --VAN_TX_RetryTime;
                    return;
                }
                //*/
            }
            if (VAN_TX_Buffer[VAN_TX_BitIndex] == 1) {
                VAN_TX_SET();
                VAN_TX_LastBitState = HIGH;
            }
            else {
                VAN_TX_CLR();
                VAN_TX_LastBitState = LOW;
            }
            ++VAN_TX_BitIndex;
        }
        else {
            // After we finished the transmission we set the bus to its initial state
            VAN_TX_SET();
            VAN_TX_LastBitState = HIGH;
            StopTXTimer();
            InitRXTimer();
        }
    }
    else {
        // If we exceeded the retry count then we set the bus to its initial state
        VAN_TX_SET();
        VAN_TX_LastBitState = HIGH;
        StopTXTimer();
        InitRXTimer();
    }
}

bool Stm32ArduinoVanBus::SendNormalFrame(uint16_t iden, const uint8_t message[], uint8_t messageLength, uint8_t requireAck)
{
    if (VAN_TX_State == VAN_SENDING || messageLength < 2) {
        return false;
    }

    uint8_t id1 = (uint8_t)(((iden << 4) & 0xff00) >> 8);
    uint8_t id2 = (uint8_t)(iden & 0xF);

    if (requireAck) {
        // EXT, RAK, RW, RTR bits of the COM field should be 1100
        id2 = id2 << 4 | 0xC;
    }
    else
    {
        // EXT, RAK, RW, RTR bits of the COM field should be 1000
        id2 = id2 << 4 | 0x8;
    }

    // Add SOF
    _BitArray_PutEManchester(VAN_TX_Buffer, 0, 0x0E);
    VAN_TX_BitCount = 10;

    // Add IDEN and COM
    _BitArray_PutEManchester(VAN_TX_Buffer, VAN_TX_BitCount, id1);
    VAN_TX_BitCount += 10;

    _BitArray_PutEManchester(VAN_TX_Buffer, VAN_TX_BitCount, id2);
    VAN_TX_BitCount += 10;

    // Add data bytes
    for (uint8_t i = 0; i < messageLength; i++)
    {
        _BitArray_PutEManchester(VAN_TX_Buffer, VAN_TX_BitCount, message[i]);
        VAN_TX_BitCount += 10;
    }

    // Add the CRC which must include the bytes of the IDEN and COM as well
    uint8_t messageWithId[34];
    messageWithId[0] = id1;
    messageWithId[1] = id2;
    memcpy(messageWithId + 2, message, messageLength);

    uint16_t crc_value = GetCrc15(messageWithId, messageLength + 2);
    _BitArray_PutEManchester(VAN_TX_Buffer, VAN_TX_BitCount, crc_value >> 8);
    VAN_TX_BitCount += 10;
    _BitArray_PutEManchester(VAN_TX_Buffer, VAN_TX_BitCount, crc_value & 0xFF);
    VAN_TX_BitCount += 10;

    // EOD
    BIT_ARRAY_CLR(VAN_TX_Buffer, VAN_TX_BitCount - 2);
    BIT_ARRAY_CLR(VAN_TX_Buffer, VAN_TX_BitCount - 1);

    VAN_TX_BitIndex = 0;
    VAN_Bus_Idle_Count = 0;
    VAN_TX_RetryTime = VAN_TX_RETRY_COUNT;
    //
    /*
    // printing to the serial port here messes up the transmission :/

    for (uint8_t counter = 0; counter < VAN_TX_BitCount; ++counter)
    {
        Serial.print(VAN_TX_Buffer[counter]);
        Serial.print(", ");
    }
        Serial.println();
    //*/

    InitTXTimer();

    return true;
}

bool Stm32ArduinoVanBus::SendQueryFrame(uint16_t iden)
{
    if (VAN_TX_State == VAN_SENDING) {
        return false;
    }

    // Add SOF
    _BitArray_PutEManchester(VAN_TX_Buffer, 0, 0x0E);
    VAN_TX_BitCount = 10;

    // Add IDEN and COM
    uint8_t id1 = (uint8_t)(((iden << 4) & 0xff00) >> 8);
    uint8_t id2 = (uint8_t)(iden & 0xF);

    //EXT, RAK, RW, RTR bits of the COM field should be 1111
    id2 = id2 << 4 | 0xF;

    _BitArray_PutEManchester(VAN_TX_Buffer, VAN_TX_BitCount, id1);
    VAN_TX_BitCount += 10;
    _BitArray_PutEManchester(VAN_TX_Buffer, VAN_TX_BitCount, id2);
    VAN_TX_BitCount += 8;

    BIT_ARRAY_SET(VAN_TX_Buffer, VAN_TX_BitCount);
    BIT_ARRAY_SET(VAN_TX_Buffer, VAN_TX_BitCount + 1);

    VAN_TX_BitIndex = 0;
    VAN_Bus_Idle_Count = 0;
    VAN_TX_RetryTime = VAN_TX_RETRY_COUNT;

    InitTXTimer();

    return true;
}

bool Stm32ArduinoVanBus::SendQueryFrameForDeferredReply(uint16_t iden)
{
    if (VAN_TX_State == VAN_SENDING) {
        return false;
    }

    // Add SOF
    _BitArray_PutEManchester(VAN_TX_Buffer, 0, 0x0E);
    VAN_TX_BitCount = 10;

    // Add IDEN and COM
    uint8_t id1 = (uint8_t)(((iden << 4) & 0xff00) >> 8);
    uint8_t id2 = (uint8_t)(iden & 0xF);

    //EXT, RAK, RW, RTR bits of the COM field should be 1111
    id2 = id2 << 4 | 0xF;

    _BitArray_PutEManchester(VAN_TX_Buffer, VAN_TX_BitCount, id1);
    VAN_TX_BitCount += 10;
    _BitArray_PutEManchester(VAN_TX_Buffer, VAN_TX_BitCount, id2);
    VAN_TX_BitCount += 10;

    uint8_t messageWithId[2];
    messageWithId[0] = id1;
    messageWithId[1] = id2;

    // Add the CRC which must include the bytes of the IDEN and COM as well
    uint16_t crc_value = GetCrc15(messageWithId, 2);
    _BitArray_PutEManchester(VAN_TX_Buffer, VAN_TX_BitCount, crc_value >> 8);
    VAN_TX_BitCount += 10;
    _BitArray_PutEManchester(VAN_TX_Buffer, VAN_TX_BitCount, crc_value & 0xFF);
    VAN_TX_BitCount += 10;

    // EOD
    BIT_ARRAY_CLR(VAN_TX_Buffer, VAN_TX_BitCount - 2);
    BIT_ARRAY_CLR(VAN_TX_Buffer, VAN_TX_BitCount - 1);

    VAN_TX_BitIndex = 0;
    VAN_Bus_Idle_Count = 0;
    VAN_TX_RetryTime = VAN_TX_RETRY_COUNT;

    InitTXTimer();

    return true;
}

uint16_t Stm32ArduinoVanBus::GetCrc15(const uint8_t message[], uint8_t messageLength)
{
    uint16_t crc = 0x7FFF;
    uint8_t  da;
    uint8_t i = 0;
    while (messageLength--)
    {
        da = crc >> 7;  // CRC(h8)

        crc <<= 8;

        crc ^= VAN_CRC15_TABLE[da ^ message[i++]];
    }
    crc ^= 0x7FFF;
    crc <<= 1;
    return crc;
}

void Stm32ArduinoVanBus::InitRXTimer()
{
    TIM_TypeDef* RXTimerInstance = TIM2;
    RX_Timer = new HardwareTimer(RXTimerInstance);
    RX_Timer->setOverflow(0xFFFF, MICROSEC_FORMAT);
    RX_Timer->attachInterrupt(VAN_RX_Timer_IT_Callback);
    RX_Timer->resume();

    attachInterrupt(digitalPinToInterrupt(_rxPin), VAN_RX_EXTI_ISR, CHANGE);
}

void Stm32ArduinoVanBus::StopRXTimer()
{
    detachInterrupt(digitalPinToInterrupt(_rxPin));

    RX_Timer->pause();
    RX_Timer->detachInterrupt();
}

void Stm32ArduinoVanBus::InitTXTimer()
{
    TIM_TypeDef* TXTimerInstance = TIM3;
    TX_Timer = new HardwareTimer(TXTimerInstance);
    TX_Timer->setOverflow(_vanNetworkType, MICROSEC_FORMAT);
    //TX_Timer->setPrescaleFactor(1);
    //TX_Timer->setOverflow(576, TICK_FORMAT);//8 us 70Mhz / 576 = 0.125
    TX_Timer->attachInterrupt(VAN_TX_Timer_IT_Callback);
    TX_Timer->resume();
}

void Stm32ArduinoVanBus::StopTXTimer()
{
    TX_Timer->pause();
    TX_Timer->detachInterrupt();
    VAN_TX_State = VAN_WAITING;
}

void Stm32ArduinoVanBus::Init(uint8_t rxPin, uint8_t txPin, uint8_t ledPin, VAN_LINE_LEVEL vanLineLevel, VAN_NETWORK_TYPE vanNetworkType)
{
    thisVanDriver = this;

    _vanLineLevel = vanLineLevel;
    _vanNetworkType = vanNetworkType;
    _rxPin = rxPin;
    _txPin = txPin;
    _ledPin = ledPin;

    pinMode(_txPin, OUTPUT);
    digitalWrite(_txPin, HIGH);

    pinMode(_rxPin, INPUT);
    if (_ledPin) {
        pinMode(_ledPin, OUTPUT);
    }

    InitRXTimer();
}

bool Stm32ArduinoVanBus::Receive(uint8_t message[], uint8_t* messageLength)
{
    if (!RxQueue.isEmpty()) {
        VAN_FRAME rec;
        if (RxQueue.lockedPop(rec)) {
            memcpy(message, rec.Data, rec.DataLen);
            *messageLength = rec.DataLen;
            return true;
        }
    }
    return false;
}

bool Stm32ArduinoVanBus::IsCrcOk(const uint8_t message[], uint8_t messageLength)
{
    uint8_t crcByte1 = message[messageLength - 2];
    uint8_t crcByte2 = message[messageLength - 1];
    uint16_t crcValueInMessage = crcByte1 << 8 | crcByte2;
    uint8_t vanMessageWithIdWithoutCrc[32];

    if (messageLength - 3 <= 32) {
        memcpy(vanMessageWithIdWithoutCrc, message + 1, messageLength - 3);
        uint16_t calculatedCrc = GetCrc15(vanMessageWithIdWithoutCrc, messageLength - 3);
        return crcValueInMessage == calculatedCrc;
    }

    return false;
}
