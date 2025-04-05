use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use tokio::sync::RwLock as TRwLock;

pub trait BlimpAlgorithm<EventType, ActionType> {
    fn handle_event(&self, ev: EventType) -> Pin<Box<impl Future<Output = ()>>>;
    fn set_action_callback(
        &mut self,
        callback: Arc<
            dyn Fn(ActionType) -> Pin<Box<dyn Future<Output = ()> + Send + Sync>> + Send + Sync,
        >,
    ) -> Pin<Box<impl Future<Output = ()>>>;
}
