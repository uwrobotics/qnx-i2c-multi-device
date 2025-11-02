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
        }

        ~JsonEncode() {
            json_encoder_destroy(enc);
        }

        std::string get_string() {
            if(buf.empty()) {
                json_encoder_end_object(enc);
                if(get_status()==JSON_ENCODER_OK) {
                    snprintf(str, max_finfo_size, "JSON:%s\n", json_encoder_buffer(enc));
                    buf = str;
                }
            }

            return buf;
        }

    private:
        json_encoder_t *enc;
        char str[1024];
        unsigned max_finfo_size = 1024;
        std::string buf;
        

        json_encoder_error_t get_status() {
            json_encoder_error_t status = json_encoder_get_status(enc);

            if ( status != JSON_ENCODER_OK ) {
                perror("Json Error");
                return status;
            }
        }
};

#endif // JSON_HELPER_H