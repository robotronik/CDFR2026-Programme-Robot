#pragma once
#include "VirtualAction.hpp"
#include "defs/tableState.hpp"
#include "constante.h"

class TakeStock : public VirtualAction {
    private:
        TableState* tableState = nullptr;
    public:
        TakeStock(const std::string& name, TableState* tableState);
        ~TakeStock() override = default;

        ReturnFSM_t run() override;
        bool stop() override;
        void reset() override;
        bool available(float &reward) override;
        bool fullBlock() override;
        bool mouvementBlock() override;

    protected:
        bool errorManagement();
        bool successManagement();
        bool besttake(const position_t& from, position_t& target);
        bool getBestQuarry(const position_t& from, position_t& target);
};