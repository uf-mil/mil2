#pragma once

#include <filesystem>
#include <iostream>

#include <boost/dll.hpp>
#include <boost/function.hpp>

namespace mil_preflight
{

class UIBase
{
  public:
    UIBase()
    {
    }

    virtual ~UIBase()
    {
    }

    virtual int spin()
    {
        return -1;
    }

    virtual void initialize([[maybe_unused]] int argc, [[maybe_unused]] char* argv[])
    {
        std::cout << error_ << std::endl;
    }

    static std::shared_ptr<UIBase> create(std::string const& uiName)
    {
        try
        {
            creator_ = boost::dll::import_alias<Creator>(uiName, uiName,
                                                         boost::dll::load_mode::append_decorations |
                                                             boost::dll::load_mode::search_system_folders);
        }
        catch (boost::system::system_error const& e)
        {
            error_ = "Failed to load the ui: " + uiName + ": " + e.code().message();
            return std::make_shared<UIBase>();
        }

        return creator_();
    }

  private:
    using Creator = std::shared_ptr<UIBase>();
    static boost::function<Creator> creator_;
    static std::string error_;
};

std::string UIBase::error_ = "success";
boost::function<UIBase::Creator> UIBase::creator_;

}  // namespace mil_preflight
