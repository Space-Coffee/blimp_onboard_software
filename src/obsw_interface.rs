use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;

pub trait BlimpAlgorithm<EventType, ActionType> {
    fn handle_event(
        self: Arc<Self>,
        ev: EventType,
    ) -> Pin<Box<dyn Future<Output = ()> + Send + Sync>>;
    fn set_action_callback(
        &self,
        callback: Arc<
            dyn Fn(ActionType) -> Pin<Box<dyn Future<Output = ()> + Send + Sync>> + Send + Sync,
        >,
    ) -> Pin<Box<impl Future<Output = ()>>>;
}
