
#include "ql_stdlib.h"
#include "ql_type.h"
#include "ql_stdlib.h"
#include "json_parser.h"

// Function to read the entire file into a string

// Helper: extract string value for a given key

int isspace(char c) {
    return (c == ' ' || c == '\n' || c == '\t' || c == '\r' || c == '\f' || c == '\v');
}


char* extract_string(const char *json, const char *key) {
    static char buffer[256];
    char pattern[64];
    Ql_snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    char *pos = Ql_strstr(json, pattern);
    if (!pos) return NULL;

    pos = Ql_strchr(pos + Ql_strlen(pattern), ':'); // find :
    if (!pos) return NULL;
    pos++;

    // Skip whitespace and quotes
    while (*pos && (isspace(*pos) || *pos == '\"')) pos++;

    char *end = Ql_strchr(pos, '\"');
    if (!end) return NULL;

    int len = end - pos;
    Ql_strncpy(buffer, pos, len);
    buffer[len] = '\0';
    return buffer;
}

// Helper: extract integer value for a given key
int extract_int(const char *json, const char *key) {
    char pattern[64];
    Ql_snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    char *pos = Ql_strstr(json, pattern);
    if (!pos) return -1;

    pos = Ql_strchr(pos + Ql_strlen(pattern), ':'); // find :
    if (!pos) return -1;
    pos++;

    // Skip whitespace
    while (*pos && isspace(*pos)) pos++;

    return Ql_atoi(pos);
}

// Helper: extract array values (very basic, strings only)
void extract_array(const char *json, const char *key) {
    char pattern[64];
    Ql_snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    char *pos = Ql_strstr(json, pattern);
    if (!pos) return;

    pos = Ql_strchr(pos, '['); // find [
    if (!pos) return;
    pos++;

    //printf("%s: ", key);

    while (*pos && *pos != ']') {
        while (*pos && (*pos == ' ' || *pos == '\"' || *pos == ',')) pos++;
        char *end = Ql_strchr(pos, '\"');
        if (!end) break;

        int len = end - pos;
        char buffer[128];
        Ql_strncpy(buffer, pos, len);
        buffer[len] = '\0';

        //printf("%s ", buffer);
        pos = end + 1;
    }
    //printf("\n");
}

/*int main() {
    char *json_data = read_file("data.json");
    if (!json_data) return 1;

    // Extract values
    char *name = extract_string(json_data, "name");
    if (name) //printf("Name: %s\n", name);

    int age = extract_int(json_data, "age");
    //printf("Age: %d\n", age);

    extract_array(json_data, "skills");

    free(json_data);
    return 0;
}*/
