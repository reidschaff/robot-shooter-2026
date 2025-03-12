/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.util;

import org.tinylog.Logger;

import java.util.*;
import java.util.function.Function;

public class ImmutableLazyOptionalMap<K, V> {
    private final List<K> keys, removedKeys = new ArrayList<>();
    private final Map<K, V> values = new HashMap<>();

    private final Function<K, Optional<V>> generator;

    public ImmutableLazyOptionalMap(List<K> keys, Function<K, Optional<V>> generator) {
        this.keys = new ArrayList<>(keys);
        this.generator = generator;
    }

    public Optional<V> get(K key) {
        if (removedKeys.contains(key)) {
            Logger.error("Initial key '{}' produced an invalid value! Removed from key list.", key);
            return Optional.empty();
        }

        if (!keys.contains(key)) {
            return Optional.empty();
        }

        if (!values.containsKey(key)) {
            Optional<V> value = generator.apply(key);
            if (value.isPresent()) {
                values.put(key, value.get());
                return value;
            } else {
                Logger.error("Initial key '{}' produced an invalid value! Removing from key list.", key);
                keys.remove(key);
                removedKeys.add(key);
                return Optional.empty();
            }
        } else {
            return Optional.of(values.get(key));
        }
    }

    public List<K> keys() {
        return List.copyOf(keys);
    }
}
