#pragma once
#include <string>

enum class ReturnFSM
{
    Working =0x0,
    Done    =0x1,
    Error   =0x2
};

class VirtualAction {
protected:
    std::string nom;
    float duree;

    /******* ERROR AND SUCCESS MANAGEMENT ******
     * These functions should be implemented to ensure errors are correctly manage 
     * */
    /*
        Manages errors for specific actions
        This prototype could be modify in subclasses but 
        error management & sucess management should not be given to main loop
    */
    //virtual bool errorManagement(NavReturn error_code) = 0;
    /*Manages the sucessful completion of the action*/
    //virtual bool successManagement() = 0;

public:
    virtual ~VirtualAction() = default;

    virtual ReturnFSM run() = 0;
    /* Return true if the action is stopped, false otherwise
    * This anticipate the case of asynchronous cancellation */
    virtual bool stop() = 0;
    /*Reset the action and all its associated resources*/
    virtual void reset() = 0;
    /* Renvoie si elle est disponible*/
    virtual bool available(float &reward) = 0;
    /*Return true if the action blocks the execution of all other actions, false otherwise*/
    virtual bool fullBlock() = 0;
    /*Return true if the action blocks the execution of movement actions, false otherwise*/
    virtual bool movementBlock() = 0;


    const std::string& getNom() const {
        return nom;
    }

    float getDuration() const {
        return duree;
    }
};