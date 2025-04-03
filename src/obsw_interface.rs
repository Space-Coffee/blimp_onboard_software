use std::future::Future;
use std::pin::Pin;

pub trait BlimpAlgorithm<EventType, ActionType> {
    fn handle_event(&self, ev: EventType) -> Pin<Box<impl Future<Output = ()>>>;
    fn set_action_callback(
        &mut self,
        callback: Box<dyn Fn(ActionType) -> Pin<Box<dyn Future<Output = ()> + Send + Sync>> + Send>,
    );
}
