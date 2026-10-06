#pragma once

#include <string>

/* Emplacement de la base des durées d'action : même dossier que les logs
   (cf. LOG_PATH dans utils/logger.hpp). */
#ifdef __CROSS_COMPILE_ARM__
    #define ACTION_DB_PATH "/home/robotronik/LOG_CDFR/action_durations.db"
#else
    #define ACTION_DB_PATH "log/action_durations.db"
#endif

struct sqlite3;

/*
    Base SQLite associant, pour chaque action terminée, son nom au temps
    qu'elle a réellement mis à s'exécuter (table `action_durations(nom, duree_ms)`).

    Le fichier est ouvert au premier enregistrement. Si l'ouverture échoue
    (dossier non inscriptible, sqlite3 absent...) l'enregistrement est
    désactivé après un log d'erreur et le FSM continue de tourner.
*/
class ActionDurationDB {
    public:
        explicit ActionDurationDB(std::string databasePath);
        ~ActionDurationDB();

        ActionDurationDB(const ActionDurationDB&) = delete;
        ActionDurationDB& operator=(const ActionDurationDB&) = delete;

        /* Ajoute une ligne (nom, dureeMs). Sans effet si la base est
           indisponible. */
        void record(const std::string& actionName, unsigned long durationMs);

    private:
        /* Ouvre la base et crée la table au besoin. Renvoie false (et
           mémorise l'échec) si la base ne peut pas être utilisée. */
        bool openIfNeeded();

        std::string path;
        sqlite3* db = nullptr;
        bool unavailable = false;
};
