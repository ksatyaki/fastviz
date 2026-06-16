fn main() {
    println!("cargo::rustc-check-cfg=cfg(ros_humble)");
    println!("cargo:rerun-if-env-changed=ROS_DISTRO");
    if let Ok(distro) = std::env::var("ROS_DISTRO") {
        if distro == "humble" {
            println!("cargo:rustc-cfg=ros_humble");
        }
    }
}
