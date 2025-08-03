#pragma once

#include <RadioLib.h>
#include <cstdint>

constexpr char LORAWAN_NAMESPACE[]     = "radiolib";
constexpr char LORAWAN_KEY_NONCES[]    = "nonces";
constexpr char LORAWAN_KEY_SESSION[]   = "session";

namespace lorawan {
int lwActivate(LoRaWANNode& node);
}
