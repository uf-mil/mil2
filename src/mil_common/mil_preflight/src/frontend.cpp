
#include "mil_preflight/ui.h"

int main(int argc, char* argv[])
{
  if (argc <= 1)
  {
    std::cout << "Usage: mil_preflight <ui> [ui_parameters...]" << std::endl;
    return -1;
  }

  std::shared_ptr<mil_preflight::UIBase> ui = mil_preflight::UIBase::create(argv[1]);

  ui->initialize(argc - 1, &argv[1]);

  return ui->spin();
}
