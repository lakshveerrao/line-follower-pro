#pragma once

#include <Arduino.h>
#include "config.h"

enum class AlgorithmMode : uint8_t {
  NormalFollow,
  LeftHandRule,
  RightHandRule,
  PathMemory,
  FloodFill,
  AStar,
  Dijkstra
};

enum class MoveChoice : char {
  Left = 'L',
  Right = 'R',
  Straight = 'S',
  Back = 'B',
  None = '-'
};

class Algorithms {
public:
  void begin();
  void setMode(AlgorithmMode mode);
  AlgorithmMode mode() const;
  const char *modeName() const;
  MoveChoice onJunction(bool left, bool straight, bool right);
  void remember(MoveChoice move);
  void simplifyRoute();
  void startReplay();
  void stopReplay();
  bool replayActive() const;
  uint8_t replayIndex() const;
  void clearRoute();
  String route() const;
  uint16_t junctionCount() const;
  String placeholderMessage() const;

private:
  AlgorithmMode _mode = AlgorithmMode::NormalFollow;
  char _route[ROUTE_MEMORY_SIZE + 1]{};
  uint8_t _routeLen = 0;
  uint8_t _replayIndex = 0;
  bool _replayActive = false;
  uint16_t _junctions = 0;
};

extern Algorithms algorithms;
