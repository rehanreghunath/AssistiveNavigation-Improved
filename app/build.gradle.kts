import java.util.Properties

// ── Read local.properties ─────────────────────────────────────────────────────
val localProps = Properties()
val localPropsFile = rootProject.file("local.properties")
if (localPropsFile.exists()) {
    localPropsFile.inputStream().use { localProps.load(it) }
}

val opencvDir = localProps.getProperty("opencv.dir")
    ?: error("opencv.dir not set in local.properties")

val openalDir = localProps.getProperty("openal.dir")
    ?: error(
        "openal.dir not set in local.properties.\n" +
                "Build OpenAL-Soft for arm64-v8a (see CMakeLists.txt for instructions)\n" +
                "then add:  openal.dir=/absolute/path/to/openal-soft"
    )

plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
}

android {
    namespace = "com.rehanreghunath.assistivenav"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.rehanreghunath.assistivenav"
        minSdk = 26
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"

        ndk {
            abiFilters += "arm64-v8a"
        }

        externalNativeBuild {
            cmake {
                arguments(
                    "-DANDROID_STL=c++_shared",
                    "-DOpenCV_DIR=$opencvDir",
                    // Pass the openal-soft root to CMakeLists so it can find
                    // the headers and the prebuilt .so.
                    "-DOPENAL_DIR=$openalDir"
                )
                cppFlags("-std=c++17")
            }
        }
    }

    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
            version = "3.22.1"
        }
    }

    sourceSets {
        getByName("main") {
            jniLibs.srcDirs(
                // OpenCV prebuilt .so files (libopencv_java4.so etc.)
                "$opencvDir/../libs",
                // OpenAL-Soft prebuilt libopenal.so
                // The directory must contain ABI subdirectories:
                //   arm64-v8a/libopenal.so
                "$openalDir/build-android"
            )
        }

        buildTypes {
            release {
                isMinifyEnabled = false
                proguardFiles(
                    getDefaultProguardFile("proguard-android-optimize.txt"),
                    "proguard-rules.pro"
                )
            }
        }

        compileOptions {
            sourceCompatibility = JavaVersion.VERSION_11
            targetCompatibility = JavaVersion.VERSION_11
        }

        kotlinOptions {
            jvmTarget = "11"
        }

        buildFeatures {
            viewBinding = true
        }
    }

    dependencies {
        implementation(libs.androidx.core.ktx)
        implementation(libs.androidx.appcompat)
        implementation(libs.material)
        implementation(libs.androidx.activity)
        implementation(libs.androidx.constraintlayout)

        implementation(libs.androidx.camera.core)
        implementation(libs.androidx.camera.camera2)
        implementation(libs.androidx.camera.lifecycle)
        implementation(libs.androidx.camera.view)
        implementation(libs.guava)

        testImplementation(libs.junit)
        androidTestImplementation(libs.androidx.junit)
        androidTestImplementation(libs.androidx.espresso.core)
    }
}