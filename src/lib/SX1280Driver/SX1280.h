#pragma once

#include "targets.h"
#include "SX1280_Regs.h"
#include "SX1280_hal.h"
#include "SX12xxDriverCommon.h"

#ifdef PLATFORM_ESP8266
#include <cstdint>
#endif

#define RADIO_SNR_SCALE 4 // Units for LastPacketSNRRaw

class SX1280Driver: public SX12xxDriverCommon
{
public:
    static SX1280Driver *instance;

    ////////////////Configuration Functions/////////////
    SX1280Driver();
    bool Begin(uint32_t minimumFrequency, uint32_t maximumFrequency);
    void End();
    void SetTxIdleMode() { SetMode(SX1280_MODE_FS, SX12XX_Radio_All); }; // set Idle mode used when switching from RX to TX
    void Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,
                uint8_t PreambleLength, bool InvertIQ, uint8_t PayloadLength,
                uint32_t flrcSyncWord=0, uint16_t flrcCrcSeed=0, uint8_t flrc=0);
    void SetFrequencyReg(uint32_t freq, SX12XX_Radio_Number_t radioNumber, bool doRx = false);
    void SetOutputPower(int8_t power);
    void startCWTest(uint32_t freq, SX12XX_Radio_Number_t radioNumber);


    bool GetFrequencyErrorbool(SX12XX_Radio_Number_t radioNumber);
    bool FrequencyErrorAvailable() const { return modeSupportsFei && (LastPacketSNRRaw > 0); }

    void TXnb(uint8_t * data, bool sendGeminiBuffer, uint8_t * dataGemini, SX12XX_Radio_Number_t radioNumber);
    void RXnb();

    uint16_t GetIrqStatus(SX12XX_Radio_Number_t radioNumber);
    void ClearIrqStatus(uint16_t irqMask, SX12XX_Radio_Number_t radioNumber);

    void GetStatus(SX12XX_Radio_Number_t radioNumber);

    uint8_t GetRxBufferAddr(SX12XX_Radio_Number_t radioNumber);
    int8_t GetRssiInst(SX12XX_Radio_Number_t radioNumber);
    void GetLastPacketStats();
    void CheckForSecondPacket();

    void ConfigureRangingCommon(uint32_t rfFrequencyHz,
                                uint8_t sf,
                                uint8_t bw,
                                uint8_t cr,
                                uint8_t preambleLength = 12);
    void SetRangingAddress(uint32_t address,
                           bool isSlave,
                           uint8_t addressBits,
                           SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_All);
    void SetRangingCalibration(uint16_t calibration,
                               SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_All);
    void SetRangingRole(SX1280_RangingRoles_t role,
                        SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_All);
    void StartRangingSlaveContinuous(SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_All);
    void StartRangingMasterOnce(SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_All);
    bool GetRangingRaw(uint32_t &rawResult,
                       SX1280_RangingResultType_t type = SX1280_RANGING_RESULT_RAW,
                       SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
    float GetRangingMeters(SX1280_RangingResultType_t type,
                           float bandwidthMHz,
                           SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
    void RestorePacketType();

private:
    // constant used for no power change pending
    // must not be a valid power register value
    static const uint8_t PWRPENDING_NONE = 0x7f;

    SX1280_RadioOperatingModes_t currOpmode;
    uint8_t packet_mode;
    bool modeSupportsFei;
    uint8_t pwrCurrent;
    uint8_t pwrPending;
    SX1280_RadioOperatingModes_t fallBackMode;
    SX1280_RadioPacketTypes_t prevPacketMode;
    bool prevPacketModeValid;
    SX12XX_Radio_Number_t lastSuccessfulPacketRadio;

    void SetMode(SX1280_RadioOperatingModes_t OPmode, SX12XX_Radio_Number_t radioNumber);
    void SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr);

    // LoRa functions
    void ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr);
    void SetPacketParamsLoRa(uint8_t PreambleLength, SX1280_RadioLoRaPacketLengthsModes_t HeaderType,
                             uint8_t InvertIQ);
    // FLRC functions
    void ConfigModParamsFLRC(uint8_t bw, uint8_t cr, uint8_t bt=SX1280_FLRC_BT_0_5);
    void SetPacketParamsFLRC(uint8_t HeaderType,
                             uint8_t PreambleLength,
                             uint32_t syncWord,
                             uint16_t crcSeed,
                             uint8_t cr);

    void SetDioIrqParams(uint16_t irqMask,
                         uint16_t dio1Mask=SX1280_IRQ_RADIO_NONE,
                         uint16_t dio2Mask=SX1280_IRQ_RADIO_NONE,
                         uint16_t dio3Mask=SX1280_IRQ_RADIO_NONE);

    static void IsrCallback_1();
    static void IsrCallback_2();
    static void IsrCallback(SX12XX_Radio_Number_t radioNumber);
    bool RXnbISR(uint16_t irqStatus, SX12XX_Radio_Number_t radioNumber); // ISR for non-blocking RX routine
    void TXnbISR(); // ISR for non-blocking TX routine
    void CommitOutputPower();

    uint8_t RangingAddressFieldFromBits(uint8_t addressBits) const;
    uint32_t FrequencyHzToReg(uint32_t freqHz) const;
    int32_t SignExtend24Bit(uint32_t value) const;
};
