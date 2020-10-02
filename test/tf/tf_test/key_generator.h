#pragma once

#define TYPES uint32_t, int32_t, std::string
#define RANGE_AND_DTYPES 2, 3, float, double

#define origin keys.at(0)
#define key_1 keys.at(1)
#define key_2 keys.at(2)
#define key_3 keys.at(3)
#define key_4 keys.at(4)
#define key_5 keys.at(5)
#define key_6 keys.at(6)
#define key_7 keys.at(7)
#define key_8 keys.at(8)
#define key_9 keys.at(9)
constexpr int keyN = 10;



//////////////////////////
/// Key Generator
//////////////////////////


template<typename T>
class KeysGenerator {

public:
    KeysGenerator(size_t size) : size_{size} {};

    std::vector<T> generateKyes() {
        std::vector<T> output;
        std::mt19937 generator(0);
        std::uniform_int_distribution<T> ragne( std::numeric_limits<T>::min() ,std::numeric_limits<T>::max());

        for (size_t i = 0 ; i < size_ ; i++) {
            output.push_back(ragne(generator));
        }

        return output;
    }

private:
    size_t size_;
};

template<>
std::vector<std::string> KeysGenerator<std::string>::generateKyes() {

    std::vector<std::string> output;
    const std::string chars = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

    std::random_device random_device;
    std::mt19937 generator(random_device());
    std::uniform_int_distribution<> distribution(0, chars.size() - 1);

    for (size_t i = 0 ; i < size_ ; i++) {
        std::string random_string;
        for (std::size_t i = 0; i < 60; ++i) {
            random_string += chars[distribution(generator)];
        }
        output.push_back(random_string);
    }
    return output;
}


template <typename T>
struct TestKeyGenerator{
    static void testFunction(size_t size){
        auto gen = KeysGenerator<T>{size};
        auto keys = gen.generateKyes();

        std::cout << std::endl << " * * * Generating keys (dtype: " << rtl::test::type<T>::description() << ") : * * * " << std::endl << std::endl;
        ASSERT_EQ(keys.size(), size);
        for(const auto& key : keys) {
            std:: cout << key << std::endl;
        }
    }
};