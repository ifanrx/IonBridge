#include "version.h"

#include <cstdint>
#include <cstdio>

Version::Version(uint8_t major, uint8_t minor, uint8_t revision)
    : major(major), minor(minor), revision(revision) {}

Version::Version(uint8_t version[3]) {
  major = version[0];
  minor = version[1];
  revision = version[2];
}

Version::Version(const std::string &versionStr) {
  unsigned int temp_major, temp_minor, temp_revision;
  sscanf(versionStr.c_str(), FORMAT, &temp_major, &temp_minor, &temp_revision);
  major = static_cast<uint8_t>(temp_major);
  minor = static_cast<uint8_t>(temp_minor);
  revision = static_cast<uint8_t>(temp_revision);
}

Version::Version(std::array<uint8_t, 3> versionArray) {
  major = versionArray[0];
  minor = versionArray[1];
  revision = versionArray[2];
}

bool Version::operator==(const Version &other) const {
  return major == other.major && minor == other.minor &&
         revision == other.revision;
}

bool Version::operator!=(const Version &other) const {
  return not(*this == other);
}

bool Version::operator>(const Version &other) const {
  if (major != other.major) {
    return major > other.major;
  }
  if (minor != other.minor) {
    return minor > other.minor;
  }
  if (revision != other.revision) {
    return revision > other.revision;
  }
  return false;
}

bool Version::operator<(const Version &other) const { return other > *this; }

std::string Version::toString() const {
  char buffer[16];
  snprintf(buffer, 16, FORMAT, major, minor, revision);
  return std::string(buffer);
}
