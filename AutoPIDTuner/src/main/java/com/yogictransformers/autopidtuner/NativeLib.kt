package com.yogictransformers.autopidtuner

class NativeLib {

    /**
     * A native method that is implemented by the 'autopidtuner' native library,
     * which is packaged with this application.
     */
    external fun stringFromJNI(): String

    companion object {
        // Used to load the 'autopidtuner' library on application startup.
        init {
            System.loadLibrary("autopidtuner")
        }
    }
}