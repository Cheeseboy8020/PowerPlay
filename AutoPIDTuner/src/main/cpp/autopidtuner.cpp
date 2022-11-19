#include <jni.h>
#include <string>
#include "AutoPIDTuner/AutoPIDTuner.h"

static AutoPIDTuner autopidtuner;

extern "C" JNIEXPORT void JNICALL
Java_com_yogictransformers_autopidtuner_NativeLib_initModel(
        JNIEnv* env,
        jobject /* this */) {
    autopidtuner.initialize();
}

extern "C" JNIEXPORT void JNICALL
Java_com_yogictransformers_autopidtuner_NativeLib_step(
        JNIEnv* env,
        jobject /* this */) {
    autopidtuner.step();
}

extern "C" JNIEXPORT void JNICALL
Java_com_yogictransformers_autopidtuner_NativeLib_stop(
        JNIEnv* env,
        jobject /* this */) {
    AutoPIDTuner::terminate();
}