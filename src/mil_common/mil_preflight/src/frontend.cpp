
#include "mil_preflight/ui.h"


int main(int argc, char* argv[]) 
{

    std::shared_ptr<mil_preflight::UIBase> ui = mil_preflight::UIBase::create("ftx_ui");

    ui->initialize(argc, argv);

    return ui->spin();
}