#ifndef JSON_HELPER_H
#define JSON_HELPER_H

#include <sys/json.h>
#include <stdint.h>
#include <string>
#include <stdlib.h>

/**
 * struct {
 * 'ID': ,1/init; 2/message
 * 'data':,}
 * slave_id:
 * Type: mem/direct (interrupt not supported)
 * 
 * size:
 * int_array:
 * )
 */


// user input: one unordered map entry (name and type and value)
// class will count the nest
// output json object


 class JsonEncode {
    public:
        JsonEncode() {
            enc = json_encoder_create();
            json_encoder_start_object(enc, NULL);
            open_brackets = 0;
        }

        ~JsonEncode() {
            json_encoder_destroy(enc);
        }

        std::string get_string() {
            while (open_brackets > 0) {
                json_encoder_end_object(enc);
                open_brackets--;
            }
            
            if(buf.empty()) {
                json_encoder_end_object(enc);
                if(get_status()==JSON_ENCODER_OK) {
                    snprintf(str, max_finfo_size, "JSON:%s\n", json_encoder_buffer(enc));
                    buf = str;
                }
            }

            return buf;
        }

        void add(std::string key, int value) {
            json_encoder_add_int(enc, key, value);
        }

        void add(std::string key, std::string value) {
            json_encoder_add_string(enc, key, value);
        }

        void indent(std::string key) {
            json_encoder_start_object(enc, key);
            open_brackets++;
        }

        void closeIndent() {
            json_encoder_end_object(enc);
            open_brackets--;
        }

    private:
        json_encoder_t *enc;
        char str[1024];
        unsigned max_finfo_size = 1024;
        std::string buf;
        int open_brackets;
        

        json_encoder_error_t get_status() {
            json_encoder_error_t status = json_encoder_get_status(enc);

            if ( status != JSON_ENCODER_OK ) {
                perror("Json Error");
                return status;
            }
        }
};

#endif // JSON_HELPER_H