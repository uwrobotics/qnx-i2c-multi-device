#ifndef JSON_HELPER_H
#define JSON_HELPER_H

#include <sys/json.h>
#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <unordered_map>
#include <vector>

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
            _enc = json_encoder_create();
            json_encoder_start_object(_enc, NULL);
            _open_brackets = 1;
        }

        ~JsonEncode() {
            json_encoder_destroy(_enc);
        }

        std::string get_string() {
            if(buf.empty()) {
                closeOpenBrackets();
                char str[1024];
                int max_finfo_size = 1024;
                if(get_status()==JSON_ENCODER_OK && max_finfo_size >= json_encoder_length(_enc)) {
                    snprintf(str, max_finfo_size, "JSON:%s\n", json_encoder_buffer(_enc));
                    buf = str;
                } else {
                    perror("Json Exceed Max Length");
                }
            }

            return buf;
        }

        void add(const std::unordered_map<std::string, int> &entry) {
            for (const auto& [key, value]: entry){
                json_encoder_add_int(_enc, key.c_str(), value);
            }
        }

        void add(const std::unordered_map<std::string, std::string> &entry) {
            for (const auto& [key, value]: entry){
                json_encoder_add_json(_enc, key.c_str(), value.c_str());
            }
        }

        void add(const std::unordered_map<std::string, std::vector<int>> entry) {
            for (const auto& [key, values]: entry){
                json_encoder_start_array(_enc, key.c_str());
                for (int value: values){
                    json_encoder_add_int(_enc, NULL, value);
                }
                json_encoder_end_array(_enc);
            }
        }

        void indent(std::string key) {
            json_encoder_start_object(_enc, key.c_str());
            _open_brackets++;
        }

        void closeIndent() {
            json_encoder_end_object(_enc);
            _open_brackets--;
        }

        int show_indent_level() {
            return _open_brackets;
        }

    private:
        json_encoder_t *_enc;
        std::string buf;
        int _open_brackets;
        

        json_encoder_error_t get_status() {
            json_encoder_error_t status = json_encoder_get_status(_enc);

            if ( status != JSON_ENCODER_OK ) {
                perror("Json Error");
            }

            return status;
        }

        void closeOpenBrackets() {
            while (_open_brackets > 0) {
                json_encoder_end_object(_enc);
                _open_brackets--;
            }
        }
};

#endif // JSON_HELPER_H