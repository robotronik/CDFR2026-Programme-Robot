#include "db/ActionDurationDB.hpp"

#include <filesystem>
#include <system_error>

#include <sqlite3.h>

#include "utils/logger.hpp"

ActionDurationDB::ActionDurationDB(std::string databasePath)
    : path(databasePath)
{
}

ActionDurationDB::~ActionDurationDB(){
    if (db != nullptr){
        sqlite3_close(db);
    }
}

bool ActionDurationDB::openIfNeeded(){
    if (db != nullptr) return true;
    if (unavailable) return false;

    // Le dossier de destination (ex. log/) peut ne pas exister au premier lancement
    std::error_code ec;
    std::filesystem::path filePath(path);
    if (filePath.has_parent_path()){
        std::filesystem::create_directories(filePath.parent_path(), ec);
    }

    if (sqlite3_open(path.c_str(), &db) != SQLITE_OK){
        LOG_ERROR("ActionDurationDB: ouverture de '", path.c_str(), "' impossible : ",
                  db != nullptr ? sqlite3_errmsg(db) : "mémoire insuffisante");
        if (db != nullptr){
            sqlite3_close(db);
            db = nullptr;
        }
        unavailable = true;
        return false;
    }

    const char* createTable =
        "CREATE TABLE IF NOT EXISTS action_durations ("
        "nom TEXT NOT NULL,"
        "duree_ms INTEGER NOT NULL);";
    char* errorMessage = nullptr;
    if (sqlite3_exec(db, createTable, nullptr, nullptr, &errorMessage) != SQLITE_OK){
        LOG_ERROR("ActionDurationDB: création de la table impossible : ",
                  errorMessage != nullptr ? errorMessage : "raison inconnue");
        sqlite3_free(errorMessage);
        sqlite3_close(db);
        db = nullptr;
        unavailable = true;
        return false;
    }

    LOG_INFO("ActionDurationDB: base ouverte '", path.c_str(), "'");
    return true;
}

void ActionDurationDB::record(const std::string& actionName, unsigned long durationMs){
    if (!openIfNeeded()) return;

    sqlite3_stmt* statement = nullptr;
    const char* insert = "INSERT INTO action_durations (nom, duree_ms) VALUES (?, ?);";
    if (sqlite3_prepare_v2(db, insert, -1, &statement, nullptr) != SQLITE_OK){
        LOG_ERROR("ActionDurationDB: requête invalide : ", sqlite3_errmsg(db));
        return;
    }

    sqlite3_bind_text(statement, 1, actionName.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int64(statement, 2, static_cast<sqlite3_int64>(durationMs));

    if (sqlite3_step(statement) != SQLITE_DONE){
        LOG_ERROR("ActionDurationDB: enregistrement de '", actionName.c_str(),
                  "' impossible : ", sqlite3_errmsg(db));
    }

    sqlite3_finalize(statement);
}
