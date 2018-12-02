#ifndef DSL_KEY_H
#define DSL_KEY_H

#pragma once

#include <common/Serialization.h>
#include <schema/gen/DSLKey_generated.h>

DECLARE_INTERNAL_SCHEMA(class DSLKey {
  public:
    SCHEMA_METHODS(DSLKey);
    SCHEMA_FIELD(int k_1);
    SCHEMA_FIELD(int k_2);

    DSLKey() : DSLKey (-1, -1) { };
    
    DSLKey(int k_1, int k_2) : k_1(k_1), k_2(k_2) { };

    inline void operator=(const DSLKey& other) {
      k_1 = other.k_1;
      k_2 = other.k_2;
    }
    
    inline bool operator==(const DSLKey& other) const {
      return (k_1 == other.k_1 && k_2 == other.k_2);
    }
    
    inline bool operator!=(const DSLKey& other) const {
      return !(*this == other);
    }
    
    inline bool operator<(const DSLKey& other) const {
      if (k_1 < other.k_1) return true;
      if (k_1 > other.k_1) return false;
      return (k_2 < other.k_2);
    }
    
    inline bool operator<=(const DSLKey& other) const {
      return (*this < other || *this == other);
    }
    
    inline bool operator>(const DSLKey& other) const {
      return !(*this <= other);
    }
    
    inline bool operator>=(const DSLKey& other) const {
      return !(*this < other);
    }
});

#endif
