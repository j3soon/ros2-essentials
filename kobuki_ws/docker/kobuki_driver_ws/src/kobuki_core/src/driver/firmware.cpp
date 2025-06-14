/**
 * @file src/driver/firmware.cpp
 *
 * @brief Constant definitions for compatible firmware versions.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <vector>
#include "../../include/kobuki_core/packets/firmware.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki {

/*****************************************************************************
 ** Constants
 *****************************************************************************/

const uint32_t Firmware::COMPATIBLE_MAJOR_VERSION = 1;
const std::vector<uint32_t> Firmware::RECOMMENDED_VERSIONS = { 0x010104, 0x010200 };

/*****************************************************************************
 ** Serialisation
 *****************************************************************************/

bool Firmware::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  unsigned char length = 4;
  buildBytes(Header::Firmware, byteStream);
  buildBytes(length, byteStream);
  buildBytes(data.version, byteStream);
  return true;
}

bool Firmware::deserialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  if (byteStream.size() < static_cast<unsigned int>(length)+2)
  {
    //std::cout << "kobuki_node: kobuki_fw: deserialise failed. not enough byte stream." << std::endl;
    return false;
  }

  unsigned char header_id(0x00), length_packed(0x00);
  buildVariable(header_id, byteStream);
  buildVariable(length_packed, byteStream);
  if( header_id != Header::Firmware ) return false;
  if( length_packed != 2 and length_packed != 4) return false;

  // TODO First 3 firmware versions coded version number on 2 bytes, so we need convert manually to our new
  // 4 bytes system; remove this horrible, dirty hack as soon as we upgrade the firmware to 1.1.2 or 1.2.0
  if (length_packed == 2)
  {
    uint16_t old_style_version = 0;
    buildVariable(old_style_version, byteStream);

    if (old_style_version == 123)
      data.version = 65536; // 1.0.0
    else if ((old_style_version == 10100) || (old_style_version == 110))
      data.version = 65792; // 1.1.0
    else if ((old_style_version == 10101) || (old_style_version == 111))
      data.version = 65793; // 1.1.1
  }
  else
  {
    buildVariable(data.version, byteStream);
  }
  // override data.version here to hack/test version checking
  // data.version = 0x010103;

  //showMe();
  return constrain();
}

/*****************************************************************************
 ** Version Helpers
 *****************************************************************************/

int Firmware::checkMajorVersion() const
{
  // Return 0 if detected major version is compatible
  // Return a positive value if the driver needs to be upgraded
  // Return a negative value if the firmware needs to be upgraded
  return majorVersion() - COMPATIBLE_MAJOR_VERSION;
}

int Firmware::checkRecommendedVersion() const
{
  // Return  0 if the version matches a recommended version
  // Return  1 if the driver needs to be upgraded
  // Return -1 if the firmware needs to be upgraded

  // look for match with a recommended version
  if ( std::find(RECOMMENDED_VERSIONS.begin(), RECOMMENDED_VERSIONS.end(), data.version) != RECOMMENDED_VERSIONS.end()) {
    return 0;
  }

  // if flashed version is > all recommended versions, return 1, else -1
  for ( uint32_t version : RECOMMENDED_VERSIONS ) {
    int recommended_major_version = VersionInfo::majorVersion(version);
    int recommended_minor_version = VersionInfo::minorVersion(version);
    int recommended_patch_version = VersionInfo::patchVersion(version);
    if ( majorVersion() > recommended_major_version ) {
      continue;
    } else if ( majorVersion() < recommended_major_version ) {
      return -1;
    } else if ( minorVersion() > recommended_minor_version ) {
      continue;
    } else if ( minorVersion() < recommended_minor_version ) {
      return -1;
    } else if ( patchVersion() > recommended_patch_version ) {
      continue;
    } else if ( patchVersion() < recommended_patch_version ) {
      return -1;
    }
  }
  // flashed version is > all recommended versions
  return 1;
}

} // namespace kobuki
