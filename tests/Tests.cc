#include "mavserver.hh"
#include <memory>

int main() {

    std::shared_ptr<MavServer> mavserver = std::make_shared<MavServer>(14558);
    mavserver->run();

    // TODO: WIP
}

