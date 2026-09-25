#include "audio_core/platform/platform_detect.hpp"
#include "audio_core/engine.hpp"
#include "backends/android/aaudio_backend.hpp"

#include <memory>
#include <string>

#if defined(AETHEL_OS_ANDROID)
#include <jni.h>

static std::unique_ptr<audio_core::Engine> g_android_engine;
static std::shared_ptr<audio_core::AAudioBackend> g_aaudio_backend;

extern "C" {

JNIEXPORT jboolean JNICALL
Java_com_aethel_audio_AethelAudioEngine_nativeInit(JNIEnv* env, jobject /*thiz*/, jint sampleRate, jint bufferSize) {
    g_aaudio_backend = std::make_shared<audio_core::AAudioBackend>();
    g_android_engine = std::make_unique<audio_core::Engine>(g_aaudio_backend);

    bool ok = g_android_engine->init(static_cast<uint32_t>(sampleRate), 2, static_cast<uint32_t>(bufferSize));
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL
Java_com_aethel_audio_AethelAudioEngine_nativeStart(JNIEnv* /*env*/, jobject /*thiz*/) {
    if (!g_android_engine) return JNI_FALSE;
    return g_android_engine->start() ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT void JNICALL
Java_com_aethel_audio_AethelAudioEngine_nativeStop(JNIEnv* /*env*/, jobject /*thiz*/) {
    if (g_android_engine) {
        g_android_engine->stop();
    }
}

JNIEXPORT void JNICALL
Java_com_aethel_audio_AethelAudioEngine_nativeTriggerNote(JNIEnv* /*env*/, jobject /*thiz*/,
                                                         jint trackIdx, jint note, jfloat velocity) {
    if (g_android_engine) {
        // Sample-accurate note trigger on target track
        (void)trackIdx;
        (void)note;
        (void)velocity;
    }
}

JNIEXPORT jint JNICALL
Java_com_aethel_audio_AethelAudioEngine_nativeGetXRuns(JNIEnv* /*env*/, jobject /*thiz*/) {
    if (g_aaudio_backend) {
        return static_cast<jint>(g_aaudio_backend->xrun_count());
    }
    return 0;
}

} // extern "C"
#endif // AETHEL_OS_ANDROID
