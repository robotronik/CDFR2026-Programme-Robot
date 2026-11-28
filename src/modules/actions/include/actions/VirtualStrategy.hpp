#pragma once
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "actions/VirtualAction.hpp"
#include "drive_interface.h"

class VirtualStrategy {
protected:
    std::string nom;
    bool isActive = false;

    // Key = action name (VirtualAction::getNom()), so an action can be
    // found/removed from the pool directly by name, O(1) on average, rather
    // than a linear search through a vector.
    std::unordered_map<std::string, std::pair<float, std::unique_ptr<VirtualAction>>> possible_actions;
    /* To add an action to the pool, in the constructor (or buildPossibleActions): */
    /*
    *    possible_actions.emplace(
    *        "ActionName",                            // must match action->getNom()
    *        std::make_pair(1.0f, std::make_unique<WaitAction>())
    *    );
    */
    std::vector<std::unique_ptr<VirtualAction>> running_actions;

public:
    virtual ~VirtualStrategy() = default;

    /* Select the best action in the pool and return a NON-OWNING pointer to
       it: the action stays owned by the strategy (it remains in the pool) and
       can therefore be replayed several times. An action remains a candidate
       as long as available() >= 0. Returns nullptr if no action is available. */
    virtual VirtualAction* bestAction() = 0;

    virtual position_t StratStartingPos() = 0;
    virtual position_t StratFinishingPos() = 0;

    void stop(){ isActive = false; }
    void resume(){ isActive = true; }

    /* Temporary ("wait") action to run when the strategy has nothing left to
       offer. The action stays owned by the strategy: the FSM only gets a
       pointer and must never free it. May return nullptr if the strategy does
       not provide one. */
    virtual VirtualAction* tempAction() = 0;

    /* Look up an action in the pool by name, without removing it.
       Returns nullptr if absent. */
    VirtualAction* findAction(const std::string& name) const {
        auto it = possible_actions.find(name);
        if (it == possible_actions.end()) return nullptr;
        return it->second.second.get();
    }

    /* Remove from the pool and return (ownership transfer) the action named
       `name`, or nullptr if absent. Useful to re-offer a specific action after
       a ReturnFSM::Done, for example. */
    std::unique_ptr<VirtualAction> extractAction(const std::string& name) {
        auto it = possible_actions.find(name);
        if (it == possible_actions.end()) return nullptr;
        std::unique_ptr<VirtualAction> action = std::move(it->second.second);
        possible_actions.erase(it);
        return action;
    }

    // NB: returned by const reference, because an unordered_map of unique_ptr
    // is not copyable.
    const std::unordered_map<std::string, std::pair<float, std::unique_ptr<VirtualAction>>>& available() const {
        return possible_actions;
    }

    const std::string& getNom() const { return nom; }
};