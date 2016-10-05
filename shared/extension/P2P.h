#ifndef EXTENSION_P2P_H
#define EXTENSION_P2P_H

#include <cstdlib>
#include <nuclear>

namespace extension {

    struct P2PListen {
        P2PListen() : hash(), reaction() {}

        std::array<uint64_t, 2> hash;
        std::shared_ptr<NUClear::threading::Reaction> reaction;
    };

    struct P2PEmit {
        P2PEmit() : target(""), hash(), data(), reliable(false) {}

        std::array<uint64_t, 2> hash;
        std::vector<char> data;
    };

    template <typename T>
    struct P2P {

        template <typename DSL, typename TFunc>
        static inline NUClear::threading::ReactionHandle bind(NUClear::Reactor& reactor, const std::string& label, TFunc&& callback) {

            auto task = std::make_unique<P2PListen>();

            task->hash = NUClear::util::serialise::Serialise<T>::hash();
            task->reaction = NUClear::util::generate_reaction<DSL, P2PListen>(reactor, label, std::forward<TFunc>(callback));

            NUClear::threading::ReactionHandle handle(task->reaction);

            reactor.powerplant.emit<NUClear::dsl::word::emit::Direct>(task);

            return handle;
        }

        template <typename DSL>
        static inline std::shared_ptr<T> get(NUClear::threading::Reaction& t) {

            auto data = store::ThreadStore<std::vector<char>>::value;

            return std::make_shared<TData>(util::serialise::Serialise<T>::deserialise(*data));
        }

        static void emit(PowerPlant&, std::shared_ptr<TData> data) {

            // Set our data in the store
            store::DataStore<TData>::set(data);

            static void emit(PowerPlant& powerplant, std::shared_ptr<TData> data) {

                auto e = std::make_unique<P2PEmit>();

                e->hash = util::serialise::Serialise<TData>::hash();
                e->data = util::serialise::Serialise<TData>::serialise(*data);

                powerplant.emit<Direct>(e);
            }
        }
    };

}

#endif //EXTENSION_P2P_H
