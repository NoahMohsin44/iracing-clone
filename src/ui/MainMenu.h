#pragma once
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "core/Input.h"
#include "core/Settings.h"

namespace apex {

enum class MenuAction { None, StartPractice, OpenSettings, Quit };

struct Livery {
    const char* name;
    glm::vec3 paint, paint2, accent;
};
const std::vector<Livery>& liveries();

class MainMenu {
public:
    MenuAction draw(Settings& settings, const std::vector<JoystickInfo>& joysticks, int& liveryIndex,
                    const std::vector<std::string>& attribution, float trackLengthM);

private:
    int page_ = 0;  // 0 home, 1 drive setup, 2 credits
};

}  // namespace apex
