#!/bin/bash

SOURCES=(
    "2_1_thread"
    "2_2_thread2"
    "2_3_datarace"
    "2_3_datarace_atomic_mod"
    "2_3_datarace_mutex_lockguard"
    "2_3_datarace_mutex_mod"
    "2_4_datarace_atomic"
    "2_5_datarace_mutex"
    "2_6_datarace_mutex_lockguard"
    "2_7_advanced_thread_with_class_mutex"
    "2_8_advanced_thread_with_class_atomic"
)

for SOURCE in ${SOURCES[@]}; do
    echo "build ... ${SOURCE}.cpp"
    g++ ${SOURCE}.cpp -o ${SOURCE} -std=c++17 -lpthread
done

