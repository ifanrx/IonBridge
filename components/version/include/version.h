#ifndef __VERSION_H
#define __VERSION_H

#include <array>
#include <cstdint>
#include <string>

class Version {
 private:
  constexpr static const char *FORMAT = "%u.%u.%u";

 public:
  explicit Version(uint8_t major, uint8_t minor, uint8_t revision);
  explicit Version(uint8_t version[3]);
  explicit Version(const std::string &versionStr);
  explicit Version(std::array<uint8_t, 3> versionArray);
  bool operator==(const Version &other) const;
  bool operator!=(const Version &other) const;
  bool operator>(const Version &other) const;
  bool operator<(const Version &other) const;
  std::string toString() const;
  std::array<uint8_t, 3> toArray() const { return {major, minor, revision}; }

  uint8_t major, minor, revision;
};

#endif  // !__VERSION_H
