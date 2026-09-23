#pragma once
#include <string>
#include <memory>
#include <vector>

#include "actions/VirtualAction.hpp"

class VirtualStrategy {
protected:
    std::string nom;
    bool status = false;
    std::vector<std::pair<float, std::unique_ptr<VirtualAction>>> possible_actions;
    /* To add action to the list you should use this syntaxe inside constructor*/
    /*
    *    possible_actions.emplace_back(
    *    1.0f, // Ponderation of action
    *    std::make_unique<WaitAction>() // Action class
    *    );
    */
    std::vector<std::unique_ptr<VirtualAction>> running_actions;

public:
    virtual ~VirtualStrategy() = default;

    virtual std::unique_ptr<VirtualAction> bestAction() = 0;

    /*Stop the strategy by blocking the run of bestAction*/
    void stop(){
        status = false;
    }

    /* Resume the strategy */
    void resume(){
        status = true;
    }

    /*Return the list of actions possible at the moment*/
    // NB: renvoyée par référence constante, car possible_actions contient
    // des std::unique_ptr : un vector de unique_ptr n'est pas copiable
    // (unique_ptr(const unique_ptr&) = delete), donc un retour par valeur
    // ne compile pas dès que cette fonction est ODR-utilisée.
    const std::vector<std::pair<float, std::unique_ptr<VirtualAction>>>& available() const {
        return possible_actions;
    }

    const std::string& getNom() const {
        return nom;
    }

};