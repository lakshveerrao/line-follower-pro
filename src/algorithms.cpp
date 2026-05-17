#include "algorithms.h"

Algorithms algorithms;

void Algorithms::begin() {
  clearRoute();
}

void Algorithms::setMode(AlgorithmMode mode) {
  _mode = mode;
}

AlgorithmMode Algorithms::mode() const {
  return _mode;
}

const char *Algorithms::modeName() const {
  switch (_mode) {
    case AlgorithmMode::NormalFollow: return "Normal";
    case AlgorithmMode::LeftHandRule: return "Left hand";
    case AlgorithmMode::RightHandRule: return "Right hand";
    case AlgorithmMode::PathMemory: return "Path memory";
    case AlgorithmMode::FloodFill: return "Flood fill";
    case AlgorithmMode::AStar: return "A*";
    case AlgorithmMode::Dijkstra: return "Dijkstra";
  }
  return "Unknown";
}

MoveChoice Algorithms::onJunction(bool left, bool straight, bool right) {
  _junctions++;
  MoveChoice move = MoveChoice::Straight;

  if (_mode == AlgorithmMode::PathMemory && _replayActive) {
    if (_replayIndex < _routeLen) {
      return static_cast<MoveChoice>(_route[_replayIndex++]);
    }
    _replayActive = false;
    return MoveChoice::Straight;
  }

  if (_mode == AlgorithmMode::LeftHandRule) {
    if (left) move = MoveChoice::Left;
    else if (straight) move = MoveChoice::Straight;
    else if (right) move = MoveChoice::Right;
    else move = MoveChoice::Back;
  } else if (_mode == AlgorithmMode::RightHandRule) {
    if (right) move = MoveChoice::Right;
    else if (straight) move = MoveChoice::Straight;
    else if (left) move = MoveChoice::Left;
    else move = MoveChoice::Back;
  }
  if (_mode == AlgorithmMode::PathMemory || _mode == AlgorithmMode::LeftHandRule || _mode == AlgorithmMode::RightHandRule) {
    remember(move);
  }
  return move;
}

void Algorithms::remember(MoveChoice move) {
  if (_routeLen >= ROUTE_MEMORY_SIZE) return;
  _route[_routeLen++] = static_cast<char>(move);
  _route[_routeLen] = '\0';
  simplifyRoute();
}

void Algorithms::simplifyRoute() {
  if (_routeLen < 3) return;
  // Basic maze simplification around a backtrack: LBR=S, LBS=R, RBL=S, SBL=R, SBS=B.
  char a = _route[_routeLen - 3];
  char b = _route[_routeLen - 2];
  char c = _route[_routeLen - 1];
  if (b != 'B') return;
  char replacement = 0;
  if (a == 'L' && c == 'R') replacement = 'B';
  else if (a == 'L' && c == 'S') replacement = 'R';
  else if (a == 'R' && c == 'L') replacement = 'B';
  else if (a == 'S' && c == 'L') replacement = 'R';
  else if (a == 'S' && c == 'S') replacement = 'B';
  if (replacement) {
    _routeLen -= 3;
    _route[_routeLen++] = replacement;
    _route[_routeLen] = '\0';
  }
}

void Algorithms::startReplay() {
  if (_routeLen == 0) return;
  _replayIndex = 0;
  _replayActive = true;
}

void Algorithms::stopReplay() {
  _replayActive = false;
  _replayIndex = 0;
}

bool Algorithms::replayActive() const {
  return _replayActive;
}

uint8_t Algorithms::replayIndex() const {
  return _replayIndex;
}

void Algorithms::clearRoute() {
  _routeLen = 0;
  _replayIndex = 0;
  _replayActive = false;
  _junctions = 0;
  _route[0] = '\0';
}

String Algorithms::route() const {
  return String(_route);
}

uint16_t Algorithms::junctionCount() const {
  return _junctions;
}

String Algorithms::placeholderMessage() const {
  if (_mode == AlgorithmMode::FloodFill || _mode == AlgorithmMode::AStar || _mode == AlgorithmMode::Dijkstra) {
    return "Requires grid map";
  }
  return "";
}
