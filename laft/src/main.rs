use nusb::MaybeFuture;

fn main() {
    for dev in nusb::list_devices().wait().unwrap() {
        println!("{:#?}", dev);
    }
}
