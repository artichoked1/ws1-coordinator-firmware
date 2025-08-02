#pragma once

#include <RadioLib.h>
#include <cstdint>

constexpr char LORAWAN_NAMESPACE[]     = "radiolib";
constexpr char LORAWAN_KEY_NONCES[]    = "nonces";
constexpr char LORAWAN_KEY_SESSION[]   = "session";
constexpr uint32_t LORAWAN_SESSION_MAGIC = 0xC0DEBEEF;

namespace lorawan {
  void restoreBuffers(LoRaWANNode& node, bool coldBoot);
  void saveSessionToRTC(LoRaWANNode& node);
  void saveToFlashIfNeeded(LoRaWANNode& node, int bootCount);
}
