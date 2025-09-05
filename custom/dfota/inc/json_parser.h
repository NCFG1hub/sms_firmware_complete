

#ifndef json_parser
#define json_parser



char* extract_string(const char *json, const char *key) ;
int extract_int(const char *json, const char *key) ;
void extract_array(const char *json, const char *key);


#endif