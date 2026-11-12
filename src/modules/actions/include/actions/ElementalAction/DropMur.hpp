#pragma once
#include <string>
#pragma once
#include <string>

class DropMur : public VirtualAction {
    public:
        DropMur();
        ~DropMur() override = default;

        ReturnFSM_t run() override;
        bool stop() override;
        void reset() override;
        bool available(float &reward) override;
        bool fullBlock() override;
        bool mouvementBlock() override;
    protected:
        bool errorManagement();
        bool successManagement();
        bool Besttake(const position_t & from, position_t& target);
        bool getBestPosition(const position_t & from, position_t& target);
    private:
    
}
