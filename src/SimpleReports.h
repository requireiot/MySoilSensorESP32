#ifndef __SimpleReports_h
#define __SimpleReports_h

void printEnvironment( Print& Serial );
void printMemoryInfo( Print& serial );
void reportEnvironmentJSON( JsonDocument& doc );
const char* reportEnvironmentString();

void printNetworkInfo( Print& serial );
void reportNetworkInfoJSON( JsonDocument& doc );
const char* reportNetworkInfoString();

#endif