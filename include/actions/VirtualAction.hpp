#pragma once
#include <string>
#include "navigation/navigation.h" //For nav_return_t

typedef enum
{
    FSM_RETURN_WORKING =0x0,
    FSM_RETURN_DONE    =0x1,
    FSM_RETURN_ERROR   =0x2
} ReturnFSM_t;

class VirtualAction {
protected:
    std::string nom;
    int duree;

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
    virtual ~VirtualAction() = default;

    virtual ReturnFSM_t run() = 0;
    /* Return true if the action is stopped, false otherwise
    * This anticipate the case of asynchronous cancellation */
    virtual bool stop() = 0;
    /*Reset the action and all its associated resources*/
    virtual void reset() = 0;
    /*Return the cost of the action if the action is available, -1 otherwise*/
    virtual float available() = 0;
    /*Return true if the action blocks the execution of all other actions, false otherwise*/
    virtual bool fullBlock() = 0;
    /*Return true if the action blocks the execution of movement actions, false otherwise*/
    virtual bool mouvementBlock() = 0;


    const std::string& getNom() const {
        return nom;
    }

    int getDuree() const {
        return duree;
    }
};