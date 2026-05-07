//! Hand-rolled CLI parser to avoid pulling clap into M0.

#[derive(Clone, Debug)]
pub struct Args {
    pub mock: bool,
    pub ros: bool,
    pub ref_frame: String,
    pub config: Option<std::path::PathBuf>,
    pub width: u32,
    pub height: u32,
}

impl Default for Args {
    fn default() -> Self {
        Args {
            mock: cfg!(feature = "mock"),
            ros: false,
            ref_frame: "map".into(),
            config: None,
            width: 1280,
            height: 800,
        }
    }
}

impl Args {
    pub fn parse() -> Self {
        let mut args = Args::default();
        let mut iter = std::env::args().skip(1);
        while let Some(arg) = iter.next() {
            match arg.as_str() {
                "--mock" => args.mock = true,
                "--no-mock" => args.mock = false,
                "--ros" => args.ros = true,
                "--no-ros" => args.ros = false,
                "--ref-frame" => {
                    if let Some(s) = iter.next() {
                        args.ref_frame = s;
                    } else {
                        eprintln!("--ref-frame requires a value");
                        std::process::exit(2);
                    }
                }
                "--config" => {
                    if let Some(s) = iter.next() {
                        args.config = Some(s.into());
                    } else {
                        eprintln!("--config requires a path");
                        std::process::exit(2);
                    }
                }
                "--width" => {
                    args.width = iter.next().and_then(|s| s.parse().ok()).unwrap_or(args.width);
                }
                "--height" => {
                    args.height = iter
                        .next()
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(args.height);
                }
                "--help" | "-h" => {
                    print_help();
                    std::process::exit(0);
                }
                other => {
                    eprintln!("unknown argument: {other}");
                    print_help();
                    std::process::exit(2);
                }
            }
        }
        args
    }
}

fn print_help() {
    println!(
        "fastviz — ROS2 visualizer\n\
         \n\
         Usage:\n  app [--mock] [--ros] [--ref-frame FRAME] [--config PATH] [--width N] [--height N]\n\
         \n\
         Controls:\n\
         \tLeft drag    orbit\n\
         \tRight drag   pan\n\
         \tScroll       zoom\n\
         \tF            reset view\n\
         \tT            top-down view\n\
         \tS            side view\n\
         \tEsc          quit\n"
    );
}
