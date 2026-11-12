#pragma once
#include "VirtualAction.hpp"
#include "defs/tableState.hpp"

class TakeStock : public VirtualAction {
    private:
        TableState* tableState;
    public:
        TakeStock(const std::string& name, TableState* tableState);
        ~TakeStock() override = default;

        bool TakeBlock(const position_t&  from, position_t& target);

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
        bool TakeBlock(const position_t&  from, position_t& target);
};