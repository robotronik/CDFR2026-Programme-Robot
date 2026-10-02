#pragma once
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "actions/VirtualAction.hpp"

class VirtualStrategy {
protected:
    std::string nom;
    bool status = false;

    // Clé = nom de l'action (VirtualAction::getNom()), pour pouvoir
    // retrouver/retirer une action du pool directement par son nom,
    // en O(1) en moyenne, plutôt que par recherche linéaire dans un vector.
    std::unordered_map<std::string, std::pair<float, std::unique_ptr<VirtualAction>>> possible_actions;
    /* Pour ajouter une action au pool, dans le constructeur (ou buildPossibleActions) : */
    /*
    *    possible_actions.emplace(
    *        "NomDeLAction",                          // doit correspondre à action->getNom()
    *        std::make_pair(1.0f, std::make_unique<WaitAction>())
    *    );
    */
    std::vector<std::unique_ptr<VirtualAction>> running_actions;

public:
    virtual ~VirtualStrategy() = default;

    virtual std::unique_ptr<VirtualAction> bestAction() = 0;

    void stop(){ status = false; }
    void resume(){ status = true; }

    virtual std::unique_ptr<VirtualAction> tempAction() = 0;

    /* Cherche une action du pool par son nom, sans la retirer.
       Renvoie nullptr si absente. */
    VirtualAction* findAction(const std::string& name) const {
        auto it = possible_actions.find(name);
        if (it == possible_actions.end()) return nullptr;
        return it->second.second.get();
    }

    /* Retire du pool et renvoie (transfert de propriété) l'action
       nommée `name`, ou nullptr si absente. Utile pour re-proposer
       une action précise après un FSM_RETURN_DONE par exemple. */
    std::unique_ptr<VirtualAction> extractAction(const std::string& name) {
        auto it = possible_actions.find(name);
        if (it == possible_actions.end()) return nullptr;
        std::unique_ptr<VirtualAction> action = std::move(it->second.second);
        possible_actions.erase(it);
        return action;
    }

    // NB: renvoyée par référence constante, car une unordered_map de
    // unique_ptr n'est pas copiable.
    const std::unordered_map<std::string, std::pair<float, std::unique_ptr<VirtualAction>>>& available() const {
        return possible_actions;
    }

    const std::string& getNom() const { return nom; }
};