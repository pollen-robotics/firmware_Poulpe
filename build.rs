use std::process::Command;
use std::{env, fs::File, io::Write, path::Path};
fn main() {
    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    // println!("cargo:rerun-if-changed=memory.x");
    
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    let feature_orbitas = ["orbita3d", "orbita2d"];

    // Check if at least one feature is enabled
    let feature_enabled = feature_orbitas
        .iter()
        .any(|feature| env::var(format!("CARGO_FEATURE_{}", feature.to_uppercase())).is_ok());

    if !feature_enabled {
        panic!(
            "At least one feature ( orbita2d or orbita3d ) must be enabled.\n
        Please specify a feature in Cargo.toml or with `cargo build --features`. 
        Example:  \n 
          cargo build --features orbita2d_pvt # for pvt version on orbita2d \n
        or \n 
         cargo build --features orbita3d_beta # for beta version on orbita3d \n"
        );
    }

    // check if dynamixel and ethercat both set and throw error
    if env::var("CARGO_FEATURE_DYNAMIXEL").is_ok() && env::var("CARGO_FEATURE_ETHERCAT").is_ok() {
        panic!(
            "\n \n Dynamixel and Ethercat features cannot be enabled at the same time. Please choose one.\n \n "
        );
    }

    // check if orbita2d and orbita3d both set and throw error
    if env::var("CARGO_FEATURE_ORBITA2D").is_ok() && env::var("CARGO_FEATURE_ORBITA3D").is_ok() {
        panic!(
            "\n \n Orbita2d and Orbita3d features cannot be enabled at the same time. Please choose one.\n \n "
        );
    }

    // Create a build time file for constants
    let out_dir = env::var("OUT_DIR").expect("No out dir");
    let dest_path = Path::new(&out_dir).join("constants.rs");
    let mut f = File::create(&dest_path).expect("Could not create file");

    // Trick to get the current commit hash and pass it to the firmware
    let output = Command::new("git")
        .args(&["rev-parse", "HEAD"])
        .output()
        .unwrap();
    let mut git_hash = String::from_utf8(output.stdout).unwrap();
    git_hash.pop(); //remove trainling '\n'
                    // println!("cargo:rustc-env=GIT_HASH={}", git_hash);
    write!(&mut f, "pub const GIT_HASH: &str = \"{}\";", git_hash).expect("Could not write file");
    // Get firmware zero values
    let zeros = option_env!("ZEROS");
    if let Some(zeros) = zeros {
        let zeros: Vec<f32> = zeros
            .split(",")
            .filter_map(|s| s.parse::<f32>().ok())
            .collect();
        if zeros.len() == 3 {
            writeln!(
                &mut f,
                "pub const HARDWARE_ZEROS: [f32;3] = [{:.32}, {:.32}, {:.32}];",
                zeros[0], zeros[1], zeros[2]
            )
            .expect("Could not write file"); // {:.32} to be sure to print the full precision. It counts...
        } else if zeros.len() == 2 {
            writeln!(
                &mut f,
                "pub const HARDWARE_ZEROS: [f32;2] = [{:.32}, {:.32}];",
                zeros[0], zeros[1]
            )
            .expect("Could not write file"); // {:.32} to be sure to print the full precision. It counts...
        } else {
            writeln!(&mut f, "pub const HARDWARE_ZEROS: [f32;3] = [0.0,0.0,0.0];")
                .expect("Could not write file");
        }
    } else {
        writeln!(
            &mut f,
            "#[cfg(feature = \"orbita2d\")]\npub const HARDWARE_ZEROS: [f32;2] = [0.0,0.0];"
        )
        .expect("Could not write file");
        writeln!(
            &mut f,
            "#[cfg(feature = \"orbita3d\")]\npub const HARDWARE_ZEROS: [f32;3] = [0.0,0.0,0.0];"
        )
        .expect("Could not write file");
    }

    // Get Dynamixel id
    let id = option_env!("DXL_ID");
    if let Some(id) = id {
        let id = id.parse::<u8>().ok().unwrap();
        writeln!(&mut f, "pub static DXL_ID: u8 = {:?};", id).expect("Could not write file");
    } else {
        writeln!(&mut f, "pub static DXL_ID: u8 = 42;").expect("Could not write file");
    }
}
