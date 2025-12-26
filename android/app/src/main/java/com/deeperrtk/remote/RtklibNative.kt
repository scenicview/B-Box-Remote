package com.deeperrtk.remote

/**
 * JNI wrapper for RTKLIB native library
 *
 * This is a stub - actual implementation requires:
 * 1. Cross-compile RTKLIB convbin/rnx2rtkp for Android ARM64
 * 2. Create JNI bindings
 * 3. Package as .so library
 *
 * For now, returns false/null to indicate PPK not available.
 * Users can export data and process on desktop with SonarGPSSync.
 */
object RtklibNative {

    private var loaded = false

    init {
        try {
            System.loadLibrary("rtklib")
            loaded = true
        } catch (e: UnsatisfiedLinkError) {
            // Library not available - PPK will be disabled
            loaded = false
        }
    }

    /**
     * Check if native library is loaded
     */
    fun isLoaded(): Boolean = loaded

    /**
     * Get RTKLIB version string
     */
    fun getVersion(): String = if (loaded) "2.4.3" else "not available"

    /**
     * Convert UBX to RINEX (convbin)
     * Returns number of epochs converted, or -1 on error
     */
    external fun convbin(
        inputPath: String,
        obsOutputPath: String,
        navOutputPath: String
    ): Int

    /**
     * Run PPK processing (rnx2rtkp)
     * Returns number of solutions, or -1 on error
     */
    external fun rnx2rtkp(
        roverObsPath: String,
        baseObsPath: String,
        navPath: String,
        outputPath: String,
        elevMask: Int
    ): Int
}
