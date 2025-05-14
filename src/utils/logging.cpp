#include "mdv/utils/logging.hpp"


//  ____            _                 _   _
// |  _ \  ___  ___| | __ _ _ __ __ _| |_(_) ___  _ __  ___
// | | | |/ _ \/ __| |/ _` | '__/ _` | __| |/ _ \| '_ \/ __|
// | |_| |  __/ (__| | (_| | | | (_| | |_| | (_) | | | \__ \
// |____/ \___|\___|_|\__,_|_|  \__,_|\__|_|\___/|_| |_|___/
//

using mdv::Logger;

//  ___                 _                           _        _   _
// |_ _|_ __ ___  _ __ | | ___ _ __ ___   ___ _ __ | |_ __ _| |_(_) ___  _ __
//  | || '_ ` _ \| '_ \| |/ _ \ '_ ` _ \ / _ \ '_ \| __/ _` | __| |/ _ \| '_ \
//  | || | | | | | |_) | |  __/ | | | | |  __/ | | | || (_| | |_| | (_) | | | |
// |___|_| |_| |_| .__/|_|\___|_| |_| |_|\___|_| |_|\__\__,_|\__|_|\___/|_| |_|
//               |_|

class VoidLogger : public mdv::Logger {
public:
    ~VoidLogger() override = default;

    void
    set_log_level(LogLevel lev) override {}

    MDV_NODISCARD LogLevel
    get_log_level() const override {
        return LogLevel::Info;
    }

private:
    void
    log_message(std::string msg, LogLevel lvl) const override {}
};

static mdv::Logger::SharedPtr default_logger = nullptr;

mdv::Logger::SharedPtr
mdv::get_void_logger() {
    static VoidLogger::SharedPtr logger = std::make_shared<VoidLogger>();
    return logger;
};

mdv::Logger::SharedPtr
mdv::get_default_logger() {
    if (default_logger == nullptr) default_logger = get_void_logger();
    return default_logger;
}

void
mdv::set_default_logger(mdv::Logger::SharedPtr logger) {
    default_logger = std::move(logger);
}
