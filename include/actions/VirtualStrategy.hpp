#pragma once
#include <string>
#include <memory>
#include <vector>

#include "actions/VirtualAction.hpp"
#include "navigation/navigation.h" //For nav_return_t

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

    /******* ERROR AND SUCCESS MANAGEMENT ******
     * These functions should be implemented to ensure errors are correctly manage 
     * */
    /*
        Manages errors for specific actions
        This prototype could be modify in subclasses but 
        error management & sucess management should not be given to main loop
    */
    //virtual bool errorManagement(nav_return_t error_code) = 0;
    /*Manages the sucessful completion of the action*/
    //virtual bool successManagement() = 0;

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
    std::vector<std::pair<float, std::unique_ptr<VirtualAction>>> available(){
        return possible_actions;
    }

    const std::string& getNom() const {
        return nom;
    }

};