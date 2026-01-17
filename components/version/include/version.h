#ifndef __VERSION_H
#define __VERSION_H

#include <array>
#include <cstdint>
#include <string>

class Version {
 private:
  constexpr static const char *FORMAT = "%u.%u.%u";

  uint8_t major, minor, revision;

 public:
  Version() : major(0), minor(0), revision(0) {}
  explicit Version(uint8_t major, uint8_t minor, uint8_t revision);
  explicit Version(uint8_t version[3]);
  explicit Version(const std::string &versionStr);
  explicit Version(std::array<uint8_t, 3> versionArray);
  bool operator==(const Version &other) const;
  bool operator!=(const Version &other) const;
  bool operator>(const Version &other) const;
  bool operator<(const Version &other) const;

  std::string ToString() const;
  std::array<uint8_t, 3> ToArray() const { return {major, minor, revision}; }

  constexpr uint8_t GetMajor() const { return major; }
  constexpr uint8_t GetMinor() const { return minor; }
  constexpr uint8_t GetRevision() const { return revision; }
};

#endif  // !__VERSION_H
