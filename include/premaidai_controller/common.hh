#ifndef _PREMAIDAI_COMMON_HH_
#define _PREMAIDAI_COMMON_HH_

#include <string>
#include <map>
#include <vector>

namespace premaidai_controller
{
    typedef std::vector<double> DoubleArray;
    typedef std::vector<std::string> StringArray;

    /**
     * @brief Joint config struct
     */
    struct JointConfig
    {
        int id;
        std::string name;
        int direction;
        int offset;
    };
    typedef std::map<int, JointConfig> JointIdConfigMap;
    typedef std::map<std::string, JointConfig> JointNameConfigMap;

    static const int EncoderMaxPosition = 11500;
    static const int EncoderMinPosition = 3500;
};

#endif // _PREMAIDAI_COMMON_HH_
